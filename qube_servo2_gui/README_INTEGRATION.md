# qube_servo2_gui — Ares v1.6

Qt6 HMI for the QUBE-Servo 2 ROS 2 / ros2_control project.

## Main control workflows

- Select `OPEN-LOOP VOLTAGE`, `PID`, or `STATE FEEDBACK`.
- PID and State Feedback support `POSITION` and `VELOCITY` reference semantics.
- Select `AUTOMATIC` or `MANUAL` as the reference source.
- `APPLY / ACTIVATE` orchestrates the selected controller through `/controller_manager`.
- `ARM REFERENCE OUTPUT` is deliberately separate from controller activation.

### Automatic reference

`REFERENCE COMMAND` contains three tabs:

- **PARAMETERS**
- **REFERENCE PREVIEW**
- **CONTROL TOPOLOGY**

This keeps the left deck compact while giving each automatic-control view the full available area.

### Manual reference

- Voltage: bipolar horizontal voltage rail.
- Position: rotary potentiometer/dial with `CAPTURE POSITION` for a bumpless handoff.
- Velocity: centered vertical aircraft-style throttle with a 0 rad/s detent.

`SAFE STOP / HOLD` and `EMERGENCY ZERO / DEACTIVATE` are immediately below the manual instrument.

## Build

```bash
cd ~/controlls_ws
rm -rf build/qube_servo2_gui install/qube_servo2_gui
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qube_servo2_gui --symlink-install
source install/setup.bash
```

## Run

Start the QUBE simulation/hardware stack and `controller_manager`, then:

```bash
ros2 launch qube_servo2_gui qube_servo2_gui.launch.py
```

The launch file sets `LC_NUMERIC=C` for this process to avoid decimal parsing problems with locales that use comma decimal separators.

## Voltage controller companion fix

If the repository's `QubeVoltageController` still returns `ERROR` from `on_deactivate()`, apply `qube_voltage_controller_companion.patch`. Controller switching requires a successful deactivation so the shared `motor_hub_link_joint/voltage` command interface can be handed to the next controller.
