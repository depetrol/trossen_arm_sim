"""Collision detection for the simulation.

MuJoCo reports every contact it resolves in ``data.contact`` but attaches no
meaning to them: a fingertip pressing a key and an elbow hitting the floor
look the same. This module reads the contacts after each physics step and
aggregates them per pair of touching geoms, so that a client can decide
which pairs matter by name and by force.

Pairs are tracked over time so that subscribers receive one ``begin`` event
when a contact starts and one ``end`` event, with duration and peak normal
force, when it has been absent for ``release_time`` seconds.
"""

import logging
import queue
import threading
from dataclasses import dataclass

import mujoco
import numpy as np

from .sim import WidowXSim

logger = logging.getLogger("trossen_arm_sim.collision")

ARM_ROOT_BODY = "base_link"


@dataclass
class Contact:
    """Aggregate of all contact points between two geoms.

    An unnamed geom is labelled by its body's name, an unnamed body by
    "body<id>", and a deformable flex by its name or "flex<id>". When the
    arm is involved, its side is reported first.
    """
    geom1: str
    body1: str
    geom2: str
    body2: str
    force: float      # total normal force over the contact points [N]
    pos: tuple[float, float, float]  # mean contact position [m]

    def to_dict(self) -> dict:
        return {
            "geom1": self.geom1,
            "body1": self.body1,
            "geom2": self.geom2,
            "body2": self.body2,
            "force": round(self.force, 4),
            "pos": [round(x, 4) for x in self.pos],
        }


@dataclass
class CollisionEvent:
    event: str  # "begin" or "end"
    time: float  # simulation time [s]
    contact: Contact
    duration: float  # time the pair has been in contact [s]
    peak_force: float  # largest normal force seen over that time [N]

    def to_dict(self) -> dict:
        return {
            "type": self.event,
            "time": round(self.time, 4),
            **self.contact.to_dict(),
            "duration": round(self.duration, 4),
            "peak_force": round(self.peak_force, 4),
        }


@dataclass
class _TrackedContact:
    contact: Contact
    start_time: float
    last_seen: float
    peak_force: float


class CollisionMonitor:
    """Tracks contacts in the scene and publishes begin/end events.

    Installs itself as a step hook on the simulation. Subscribers get a
    queue of CollisionEvent; active contacts can also be polled.
    """

    def __init__(self, sim: WidowXSim, release_time: float = 0.02):
        self.sim = sim
        self.release_time = release_time
        model = sim.model

        root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ARM_ROOT_BODY)
        assert root >= 0, f"body '{ARM_ROOT_BODY}' not found in scene"
        self._is_robot_body = np.zeros(model.nbody, dtype=bool)
        for body in range(model.nbody):
            ancestor = body
            while ancestor > 0 and ancestor != root:
                ancestor = model.body_parentid[ancestor]
            self._is_robot_body[body] = ancestor == root

        self._body_labels = [model.body(i).name or f"body{i}"
                             for i in range(model.nbody)]
        self._geom_labels = [
            model.geom(i).name or self._body_labels[model.geom_bodyid[i]]
            for i in range(model.ngeom)
        ]
        self._flex_labels = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_FLEX, i) or f"flex{i}"
                             for i in range(model.nflex)]

        self._lock = threading.Lock()
        self._time = 0.0
        self._tracked: dict[tuple[str, str], _TrackedContact] = {}
        self._subscribers: list[queue.Queue] = []
        self._force = np.zeros(6)
        sim.add_step_hook(self._on_step)

    @property
    def time(self) -> float:
        """Simulation time of the last processed step [s]."""
        with self._lock:
            return self._time

    def active_contacts(self) -> list[Contact]:
        with self._lock:
            return [tracked.contact for tracked in self._tracked.values()]

    def subscribe(self) -> tuple[list[Contact], queue.Queue]:
        """Start receiving events; returns the active contacts at that moment
        and the queue the following events will be delivered to."""
        with self._lock:
            events: queue.Queue = queue.Queue()
            self._subscribers.append(events)
            return [tracked.contact for tracked in self._tracked.values()], events

    def unsubscribe(self, events: queue.Queue) -> None:
        with self._lock:
            self._subscribers.remove(events)

    # ------------------------------------------------------------ step hook

    def _on_step(self) -> None:
        """Runs on the stepping thread with the simulation lock held."""
        data = self.sim.data
        current = self._read_contacts() if data.ncon > 0 else {}
        with self._lock:
            self._time = data.time
            if not current and not self._tracked:
                return
            for key, contact in current.items():
                tracked = self._tracked.get(key)
                if tracked is None:
                    self._tracked[key] = _TrackedContact(
                        contact, data.time, data.time, contact.force)
                    self._publish("begin", contact, 0.0, contact.force)
                else:
                    tracked.contact = contact
                    tracked.last_seen = data.time
                    tracked.peak_force = max(tracked.peak_force, contact.force)
            for key in [k for k, t in self._tracked.items()
                        if k not in current
                        and data.time - t.last_seen >= self.release_time]:
                tracked = self._tracked.pop(key)
                self._publish("end", tracked.contact,
                              tracked.last_seen - tracked.start_time,
                              tracked.peak_force)

    def _read_contacts(self) -> dict[tuple[str, str], Contact]:
        """Aggregate this step's contact points per geom pair."""
        model, data = self.sim.model, self.sim.data
        ncon = data.ncon
        geoms = data.contact.geom[:ncon]
        flexes = data.contact.flex[:ncon]
        # A flex side has geom -1; it is never part of the arm.
        is_robot = (geoms >= 0) & self._is_robot_body[model.geom_bodyid[geoms]]
        positions = data.contact.pos[:ncon]

        sums: dict[tuple[str, str], list] = {}
        for i in range(ncon):
            mujoco.mj_contactForce(model, data, i, self._force)
            # Arm side first; otherwise keep MuJoCo's order.
            first = 1 if is_robot[i, 1] and not is_robot[i, 0] else 0
            geom1, body1 = self._labels(geoms[i, first], flexes[i, first])
            geom2, body2 = self._labels(geoms[i, 1 - first], flexes[i, 1 - first])
            entry = sums.setdefault((geom1, geom2), [body1, body2, 0.0, np.zeros(3), 0])
            entry[2] += self._force[0]
            entry[3] += positions[i]
            entry[4] += 1
        return {
            key: Contact(key[0], body1, key[1], body2, float(force),
                         tuple(map(float, pos_sum / count)))
            for key, (body1, body2, force, pos_sum, count) in sums.items()
        }

    def _labels(self, geom: int, flex: int) -> tuple[str, str]:
        """(geom label, body label) of one side of a contact."""
        if geom >= 0:
            return self._geom_labels[geom], self._body_labels[self.sim.model.geom_bodyid[geom]]
        return self._flex_labels[flex], self._flex_labels[flex]

    def _publish(self, event: str, contact: Contact, duration: float,
                 peak_force: float) -> None:
        """Called with self._lock held."""
        logger.debug("contact %s: %s <-> %s (%.2f N peak, %.2f s)", event,
                     contact.geom1, contact.geom2, peak_force, duration)
        collision = CollisionEvent(event, self._time, contact, duration, peak_force)
        for events in self._subscribers:
            events.put(collision)
