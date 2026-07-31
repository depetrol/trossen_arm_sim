"""Run the simulated Trossen arm controller.

Usage:
    python -m trossen_arm_sim [SCENE] [--viewer] [--verbose]

SCENE is a MuJoCo MJCF file that includes the packaged arm model via
`<include file="WIDOWX_XML"/>`; the WIDOWX_XML and TROSSEN_ARM_ASSET_DIR
placeholders are replaced with the installed asset paths at load time. It
defaults to a built-in empty scene with just the arm on a ground plane.
With --viewer, a MuJoCo viewer window shows the scene (on macOS this
requires running under mjpython).
"""

import argparse
import logging
import time

from .server import TrossenArmSimServer
from .sim import DEFAULT_SCENE, WidowXSim


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", nargs="?", default=DEFAULT_SCENE,
                        help="MuJoCo scene XML including the arm model "
                             "(default: built-in empty scene)")
    parser.add_argument("--host", default="0.0.0.0", help="bind address")
    parser.add_argument("--viewer", action="store_true",
                        help="show the MuJoCo viewer (macOS: run with mjpython)")
    parser.add_argument("--verbose", action="store_true",
                        help="log every protocol request")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
    )

    sim = WidowXSim(args.scene)
    sim.start()
    server = TrossenArmSimServer(sim, host=args.host)
    server.start()

    try:
        if args.viewer:
            sim.sync_viewer_loop()
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        sim.stop()


if __name__ == "__main__":
    main()
