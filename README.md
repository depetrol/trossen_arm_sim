# trossen_arm_sim

Runs `trossen_arm` in simulation. The server speaks the wire protocol of the
WidowX AI (wxai_v0) arm controller (firmware v1.10), so the unmodified
`trossen_arm` driver connects to it as if it were real hardware — the arm
runs in MuJoCo instead.

## Install

```bash
pip install .
```

Clients need `trossen_arm==1.10.*` (the driver rejects other firmware
versions).

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

A custom scene is MJCF that includes the arm via the `WIDOWX_XML`
placeholder, replaced with the installed arm model path at load time
(`TROSSEN_ARM_ASSET_DIR` likewise for the packaged asset directory):

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

Numpad scene plus a client that presses its keys:

```bash
trossen-arm-sim examples/scene_numpad.xml     # terminal 1
python examples/press_button.py 1 2 3 enter   # terminal 2
```

## Publish

Bump `version` in `pyproject.toml`, then:

```bash
pip install build twine
python -m build
twine upload dist/*
```

## Attributions

The arm model and meshes in `trossen_arm_sim/assets/` are from
[trossen_arm_mujoco](https://github.com/TrossenRobotics/trossen_arm_mujoco),
Copyright (c) 2025 Trossen Robotics, licensed under BSD-3-Clause.
