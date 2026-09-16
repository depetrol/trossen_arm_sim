import argparse
import json
import threading
from fnmatch import fnmatchcase

import numpy as np
import trossen_arm
from websockets.sync.client import connect

ARM_BODIES = ["base_link", "link_*", "carriage_*"]
GOAL_TIME = 2.0  # seconds per move

HOME = np.zeros(6)
# Arm raised and rotated about the base: clear of the numpad in scene_numpad.xml.
WAVE_LEFT = np.array([0.8, 0.6, 0.6, 0.0, 0.0, 0.0])
WAVE_RIGHT = np.array([-0.8, 0.6, 0.6, 0.0, 0.0, 0.0])
# Shoulder pitched all the way forward: the gripper is driven into the floor.
CRASH = np.array([0.0, 2.9, 0.0, 0.0, 0.0, 0.0])


def label(event: dict) -> str | None:
    """Client-side classification; None means not shown."""
    geom1, geom2 = event["geom1"], event["geom2"]
    if {geom1, geom2} == {"carriage_left", "carriage_right"}:
        return None  # the fingers touch whenever the gripper closes
    if not any(fnmatchcase(event["body1"], p) for p in ARM_BODIES):
        return None  # scene objects touching each other
    if geom1.startswith("carriage_") and geom2.startswith("key_"):
        return "PRESS"
    return "CRASH"


def watch(host: str, port: int, show_all: bool) -> None:
    """Print collision events until the connection closes."""
    with connect(f"ws://{host}:{port}") as websocket:
        for message in websocket:
            event = json.loads(message)
            if event["type"] == "snapshot":
                continue
            kind = "" if show_all else label(event)
            if kind is None:
                continue
            print(f"\r{kind:5s} {event['time']:8.3f}s {event['type']:5s} "
                  f"{event['geom1']} ({event['body1']}) <-> "
                  f"{event['geom2']} ({event['body2']}) "
                  f"force={event['force']:.2f}N peak={event['peak_force']:.2f}N "
                  f"duration={event['duration']:.2f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1",
                        help="simulator address")
    parser.add_argument("--port", type=int, default=50002,
                        help="collision stream WebSocket port")
    parser.add_argument("--all", action="store_true",
                        help="print every contact pair unfiltered")
    args = parser.parse_args()

    threading.Thread(target=watch, args=(args.host, args.port, args.all),
                     daemon=True, name="collision-watch").start()

    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        args.host,
        False,
    )
    driver.set_all_modes(trossen_arm.Mode.position)
    print("Moving to home position...")
    driver.set_arm_positions(HOME, GOAL_TIME, True)
    driver.set_gripper_position(0.0, GOAL_TIME, True)

    try:
        while True:
            command = input("wave / crash / quit: ").strip().lower()
            if command == "wave":
                for target in (WAVE_LEFT, WAVE_RIGHT, WAVE_LEFT, HOME):
                    driver.set_arm_positions(target, GOAL_TIME, True)
            elif command == "crash":
                driver.set_arm_positions(CRASH, GOAL_TIME, True)
                driver.set_arm_positions(HOME, GOAL_TIME, True)
            elif command in ("quit", "q", "exit"):
                break
            elif command:
                print(f"Unknown command '{command}'")
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        print("Returning home...")
        driver.set_arm_positions(HOME, GOAL_TIME, True)
        driver.set_all_modes(trossen_arm.Mode.idle)


if __name__ == "__main__":
    main()
