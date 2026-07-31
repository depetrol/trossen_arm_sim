"""MuJoCo simulation backend for the WidowX AI arm."""

import os
import tempfile
import threading
import time

import mujoco
import numpy as np

from .protocol import JointOutput, Mode

ASSETS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
ARM_MODEL_PATH = os.path.join(ASSETS_DIR, "wxai_follower.xml")
DEFAULT_SCENE = os.path.join(ASSETS_DIR, "scene_empty.xml")

# Scene files reference the packaged assets through these placeholders, which
# are substituted with the installed paths before MuJoCo parses the XML.
PLACEHOLDERS = {
    "WIDOWX_XML": ARM_MODEL_PATH,
    "TROSSEN_ARM_ASSET_DIR": ASSETS_DIR,
}

ARM_JOINT_NAMES = [f"joint_{i}" for i in range(6)]
GRIPPER_JOINT_NAME = "left_carriage_joint"
JOINT_NAMES = ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
ACTUATOR_NAMES = ARM_JOINT_NAMES + ["left_gripper"]


def load_scene(scene_path: str) -> mujoco.MjModel:
    """Load a MJCF scene, substituting the WIDOWX_XML and
    TROSSEN_ARM_ASSET_DIR placeholders with the installed asset paths."""
    with open(scene_path) as f:
        xml = f.read()
    resolved = xml
    for placeholder, path in PLACEHOLDERS.items():
        resolved = resolved.replace(placeholder, path)
    if resolved == xml:
        return mujoco.MjModel.from_xml_path(scene_path)
    # Load from a resolved copy next to the scene so the scene's own
    # relative includes and asset paths still resolve.
    scene_dir = os.path.dirname(os.path.abspath(scene_path))
    fd, resolved_path = tempfile.mkstemp(dir=scene_dir, suffix=".xml")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(resolved)
        return mujoco.MjModel.from_xml_path(resolved_path)
    finally:
        os.unlink(resolved_path)


class WidowXSim:
    """Real-time MuJoCo simulation of a WidowX AI arm.

    The scene is MJCF that includes the packaged arm model via the
    WIDOWX_XML placeholder (see load_scene); it may contain any other
    bodies, joints and actuators. By default the arm stands alone on a
    ground plane (DEFAULT_SCENE). Physics steps run in a background thread
    paced to wall-clock time. Position commands set the arm's
    position-actuator targets; joint states are read back from the
    simulated joints.
    """

    def __init__(self, scene_path: str = DEFAULT_SCENE):
        self.model = load_scene(scene_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

        self._qpos_addr = np.array([
            self.model.jnt_qposadr[self._name2id(mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in JOINT_NAMES
        ])
        self._qvel_addr = np.array([
            self.model.jnt_dofadr[self._name2id(mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in JOINT_NAMES
        ])
        self._actuator_ids = np.array([
            self._name2id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        ])
        self.num_joints = len(JOINT_NAMES)

        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

    def _name2id(self, obj_type: mujoco.mjtObj, name: str) -> int:
        obj_id = mujoco.mj_name2id(self.model, obj_type, name)
        assert obj_id >= 0, \
            f"{obj_type.name} '{name}' not found in scene; " \
            f"the scene must include the arm model ({ARM_MODEL_PATH})"
        return obj_id

    @property
    def joint_limits(self) -> list[tuple[float, float]]:
        """(min, max) actuator range per joint, arm joints then gripper."""
        return [tuple(self.model.actuator_ctrlrange[i]) for i in self._actuator_ids]

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._step_loop, daemon=True,
                                        name="mujoco-step")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    def _step_loop(self) -> None:
        dt = self.model.opt.timestep
        next_step = time.perf_counter()
        while self._running:
            with self._lock:
                mujoco.mj_step(self.model, self.data)
            next_step += dt
            delay = next_step - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_step = time.perf_counter()  # fell behind; do not spiral

    def apply_joint_commands(self, modes: list[Mode], positions: list[float]) -> None:
        """Set actuator targets for joints commanded in position mode."""
        with self._lock:
            for i in range(self.num_joints):
                if modes[i] == Mode.POSITION:
                    actuator = self._actuator_ids[i]
                    low, high = self.model.actuator_ctrlrange[actuator]
                    self.data.ctrl[actuator] = np.clip(positions[i], low, high)

    def read_joint_outputs(self) -> list[JointOutput]:
        with self._lock:
            qpos = self.data.qpos[self._qpos_addr].copy()
            qvel = self.data.qvel[self._qvel_addr].copy()
            force = self.data.actuator_force[self._actuator_ids].copy()
        return [
            JointOutput(position=float(qpos[i]), velocity=float(qvel[i]),
                        effort=float(force[i]))
            for i in range(self.num_joints)
        ]

    def sync_viewer_loop(self) -> None:
        """Run a passive viewer until its window is closed.

        Must be called from the main thread; on macOS this requires running
        under `mjpython`.
        """
        import mujoco.viewer
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and self._running:
                with self._lock:
                    viewer.sync()
                time.sleep(1 / 60)
