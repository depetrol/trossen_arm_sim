"""Run the simulated Trossen arm controller.

Usage:
    python -m trossen_arm_sim [SCENE] [--viewer] [--verbose] [--ws-port PORT]

SCENE is a MuJoCo MJCF file that includes the packaged arm model via
`<include file="WIDOWX_XML"/>`; the WIDOWX_XML and TROSSEN_ARM_ASSET_DIR
placeholders are replaced with the installed asset paths at load time. It
defaults to a built-in empty scene with just the arm on a ground plane.
With --viewer, a MuJoCo viewer window shows the scene (on macOS this
requires running under mjpython).

Contacts in the scene are streamed as JSON over a WebSocket on --ws-port;
see collision_server.py for the messages.
"""

import argparse
import logging
import time

from .collision import CollisionMonitor
from .collision_server import WS_PORT, CollisionWebSocketServer
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
    parser.add_argument("--ws-port", type=int, default=WS_PORT,
                        help="WebSocket port for the collision stream")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
    )

    sim = WidowXSim(args.scene)
    monitor = CollisionMonitor(sim)
    sim.start()
    server = TrossenArmSimServer(sim, host=args.host)
    server.start()
    collision_server = CollisionWebSocketServer(monitor, host=args.host,
                                                port=args.ws_port)
    collision_server.start()

    try:
        if args.viewer:
            sim.sync_viewer_loop()
        else:
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        collision_server.stop()
        server.stop()
        sim.stop()


if __name__ == "__main__":
    main()
