"""Press numpad keys with the simulated arm using the trossen_arm driver.

Start the simulator first, in another terminal — headless:

    trossen-arm-sim examples/scene_numpad.xml

or with the MuJoCo viewer (macOS needs mjpython):

    mjpython -m trossen_arm_sim examples/scene_numpad.xml --viewer

Then run this client (needs trossen_arm==1.10.*):

    python examples/press_button.py 1 2 3 enter
    python examples/press_button.py              # interactive prompt

Joint positions for each key were recorded against the numpad pose in
scene_numpad.xml and are stored in positions.json.
"""

import argparse
import json
import os

import numpy as np
import trossen_arm

POSITIONS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "positions.json")
GOAL_TIME = 2.0  # seconds per move


def move_home(driver: trossen_arm.TrossenArmDriver) -> None:
    driver.set_arm_positions(np.zeros(driver.get_num_joints() - 1),
                             GOAL_TIME, True)
    driver.set_gripper_position(0.0, GOAL_TIME, True)


def press_key(driver: trossen_arm.TrossenArmDriver, key: str,
              key_positions: dict) -> None:
    hover = np.array(key_positions[key]["hover"][:6])
    press = np.array(key_positions[key]["press"][:6])
    print(f"Pressing '{key}'...")
    for target in (hover, press, hover):
        driver.set_arm_positions(target, GOAL_TIME, True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("keys", nargs="*",
                        help="keys to press (default: interactive prompt)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="controller address (the simulator, or a real arm)")
    args = parser.parse_args()

    with open(POSITIONS_PATH) as f:
        key_positions = json.load(f)["key_positions"]

    for key in args.keys:
        if key not in key_positions:
            parser.error(f"unknown key '{key}'; "
                         f"available: {', '.join(key_positions)}")

    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        args.host,
        False,
    )
    driver.set_all_modes(trossen_arm.Mode.position)
    print("Moving to home position...")
    move_home(driver)

    try:
        if args.keys:
            for key in args.keys:
                press_key(driver, key, key_positions)
        else:
            print(f"Available keys: {', '.join(key_positions)}")
            while True:
                key = input("Key to press (Ctrl+C to exit): ").strip()
                if key in key_positions:
                    press_key(driver, key, key_positions)
                elif key:
                    print(f"Unknown key '{key}'. "
                          f"Available: {', '.join(key_positions)}")
    except KeyboardInterrupt:
        print()
    finally:
        print("Returning home...")
        move_home(driver)
        driver.set_all_modes(trossen_arm.Mode.idle)


if __name__ == "__main__":
    main()
