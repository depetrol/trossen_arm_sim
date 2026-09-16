"""WebSocket server streaming collision events as JSON."""

import json
import logging
import queue
import threading

from websockets.exceptions import ConnectionClosed
from websockets.sync.server import Server, ServerConnection, serve

from .collision import CollisionMonitor

logger = logging.getLogger("trossen_arm_sim.collision")

WS_PORT = 50002


class CollisionWebSocketServer:
    """Streams CollisionMonitor events to every connected WebSocket client.

    On connect a client receives one ``snapshot`` message with the contacts
    active at that moment, then a ``begin`` or ``end`` message per collision
    event (see CollisionEvent.to_dict). Messages sent by clients are ignored.
    """

    def __init__(self, monitor: CollisionMonitor, host: str = "0.0.0.0",
                 port: int = WS_PORT):
        self.monitor = monitor
        self.host = host
        self.port = port
        self._server: Server | None = None
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._queues: set[queue.Queue] = set()

    def start(self) -> None:
        self._server = serve(self._handle, self.host, self.port)
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        daemon=True, name="collision-ws")
        self._thread.start()
        logger.info("Collision stream listening on ws://%s:%d", self.host, self.port)

    def stop(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        with self._lock:
            for events in self._queues:
                events.put(None)  # wakes the handler so it closes its connection
        self._thread.join()
        self._server = None
        self._thread = None

    def _handle(self, websocket: ServerConnection) -> None:
        logger.info("Collision stream client connected: %s", websocket.remote_address)
        snapshot, events = self.monitor.subscribe()
        with self._lock:
            self._queues.add(events)
        try:
            websocket.send(json.dumps({
                "type": "snapshot",
                "time": round(self.monitor.time, 4),
                "contacts": [contact.to_dict() for contact in snapshot],
            }))
            while (event := events.get()) is not None:
                websocket.send(json.dumps(event.to_dict()))
            websocket.close()
        except ConnectionClosed:
            pass
        finally:
            with self._lock:
                self._queues.discard(events)
            self.monitor.unsubscribe(events)
            logger.info("Collision stream client disconnected")
