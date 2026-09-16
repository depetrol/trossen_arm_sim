# trossen_arm_sim

Runs `trossen_arm` in simulation. The server speaks the wire protocol of the WidowX AI (wxai_v0) arm controller (firmware v1.10), so the unmodified `trossen_arm` driver connects to it as if it were real hardware — the arm runs in MuJoCo instead.

## Install

```bash
pip install .
```

Clients need `trossen_arm==1.10.*` (the driver rejects other firmware versions).

## Usage

```bash
trossen-arm-sim                        # default scene: arm on a ground plane
trossen-arm-sim scene.xml              # custom scene
mjpython -m trossen_arm_sim --viewer   # MuJoCo viewer (macOS needs mjpython)
```

Point any `trossen_arm` client at `127.0.0.1` instead of the robot's IP:

```python
driver.configure(
    trossen_arm.Model.wxai_v0,
    trossen_arm.StandardEndEffector.wxai_v0_follower,
    "127.0.0.1",
    False,
)
```

A custom scene is MJCF that includes the arm via the `WIDOWX_XML` placeholder, replaced with the installed arm model path at load time (`TROSSEN_ARM_ASSET_DIR` likewise for the packaged asset directory):

```xml
<mujoco>
  <include file="WIDOWX_XML"/>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" rgba="0.5 0.5 0.5 1"/>
  </worldbody>
</mujoco>
```

## Example

Numpad scene plus a client that presses its keys and prints every collision the simulator reports (see [Collision stream](#collision-stream) below):

```bash
trossen-arm-sim examples/scene_numpad.xml     # terminal 1
python examples/press_button.py 1 2 3 enter   # terminal 2
```

## Collision stream

The simulator streams contacts between geoms over a WebSocket, so a client can tell when the arm touches something, what it touched, and how hard.

### Connecting

Connect to `ws://<host>:50002` (`--ws-port` changes the port). The server sends JSON text messages and ignores anything the client sends. Any WebSocket library works; with `websockets`:

```python
import json
from websockets.sync.client import connect

with connect("ws://127.0.0.1:50002") as ws:
    for message in ws:
        event = json.loads(message)
        print(event["type"], event.get("geom1"), event.get("geom2"))
```

### Messages

| `type` | When | Content |
|---|---|---|
| `snapshot` | Once, right after connecting | `time` and `contacts`, the list of contact pairs active at that moment (each with the fields below except `type`, `time`, `duration`, `peak_force`) |
| `begin` | A pair of geoms starts touching | The contact fields; `duration` is 0 and `peak_force` equals `force` |
| `end` | The pair has not touched for 20 ms | The contact as last seen; `duration` and `peak_force` cover the whole contact |

```json
{"type": "snapshot", "time": 3.2, "contacts": []}
{"type": "begin", "time": 10.736, "geom1": "carriage_right", "body1": "carriage_right", "geom2": "key_5", "body2": "numpad", "force": 0.78, "pos": [0.39, 0.296, 0.026], "duration": 0.0, "peak_force": 0.78}
{"type": "end", "time": 11.404, "geom1": "carriage_right", "body1": "carriage_right", "geom2": "key_5", "body2": "numpad", "force": 0.0, "pos": [0.39, 0.296, 0.026], "duration": 0.65, "peak_force": 0.78}
{"type": "begin", "time": 15.1, "geom1": "link_6", "body1": "link_6", "geom2": "floor", "body2": "world", "force": 80.2, "pos": [0.378, -0.038, 0.0], "duration": 0.0, "peak_force": 80.2}
```

| Field | Unit | Meaning |
|---|---|---|
| `time` | s | Simulation time of the message |
| `geom1`, `geom2` | | The two touching geoms (see naming below) |
| `body1`, `body2` | | The bodies those geoms belong to; `world` for the floor |
| `force` | N | Total normal force between the pair at the time of the message |
| `peak_force` | N | Largest `force` seen since the pair started touching |
| `pos` | m | Mean position of the contact points, world frame |
| `duration` | s | How long the pair has been touching |

### Naming

A contact pair is identified by two geom labels. A named geom is labelled with its name. An unnamed geom is labelled with its body's name, so all the unnamed collision geoms of one body aggregate into one pair. An unnamed body is labelled `body<id>`, and a deformable flex with its name. Name the geoms you want to tell apart, as `scene_numpad.xml` does with `key_*`.

When the arm is involved, it is always side 1: `geom1`/`body1` is one of `base_link`, `link_1` to `link_6`, `carriage_left` or `carriage_right` (the gripper fingers). Contacts between two arm parts, and between scene objects that do not involve the arm, are streamed as well, in MuJoCo's order.

### Filtering on the client

Everything MuJoCo resolves is streamed, including the two gripper fingers touching each other whenever the gripper closes and objects resting on the floor. The client decides what matters:

- **By name**: `geom1` starting with `carriage_` and `geom2` starting with `key_` is a key press; `geom2 == "floor"` is a crash; a key touched by `link_2` is a crash too, so check both sides.
- **By force**: a press and a smash on the same key differ only in `force` and `peak_force`. Pressing a numpad key reads about 1 N; a finger driven into the floor reads hundreds of N.

`examples/watch_collisions.py` does exactly this: it prints the stream on a background thread, labelling each event PRESS or CRASH (`--all` prints every pair unfiltered), while the prompt drives the arm through the `trossen_arm` driver. `wave` swings the arm clear of everything and prints nothing; `crash` drives the gripper into the floor and prints CRASH events with the impact force; `quit` returns home.

```bash
python examples/watch_collisions.py
```

## Publish

Bump `version` in `pyproject.toml`, then:

```bash
pip install build twine
python -m build
twine upload dist/*
```

## Attributions

The arm model and meshes in `trossen_arm_sim/assets/` are from [trossen_arm_mujoco](https://github.com/TrossenRobotics/trossen_arm_mujoco), Copyright (c) 2025 Trossen Robotics, licensed under BSD-3-Clause.
