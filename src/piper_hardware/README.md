# PiPER measured-feedback hardware interface (ROS 2 Jazzy)

The real MoveIt configurations use `piper_hardware/PiperSystem`. `read()` exposes
measured joint positions and velocities; `write()` publishes setpoints on a
separate topic. It never copies a command into measured state.

```
JTC -> PiperSystem.write -> /piper/joint_commands -> piper_single_ctrl -> CAN
CAN -> passive piper_read_slave_joint -> /joint_states -> PiperSystem.read -> JTC
```

`/joint_state_broadcaster/joint_states` is now measured state, **not a command**.
There is one publisher of global `/joint_states`: the passive CAN reader. The
hardware interface also requires `/piper/hardware_ready`. Simulation may explicitly
select `use_mock_hardware:=true` in the MoveIt xacro; Gazebo's own hardware is unchanged.
Do not connect mock broadcaster state to a real arm.

## Startup and faults

Start the standard `piper/start_single_piper.launch.py` driver before the real
MoveIt launch. The hardware waits up to five seconds for complete, fresh, healthy
measurements, then initializes command interfaces from the measured pose. YAML
initial zeros are used only by explicit mock hardware.

The reader tracks kernel receive timestamps individually for position pairs
0x2A5–0x2A7, motor velocity 0x251–0x256, driver status 0x261–0x266 and arm status
0x2A1 (plus gripper 0x2A8 when configured). A ROS timer cannot refresh stale CAN
positions. Source and monotonic receipt age are checked again by the hardware
interface; the default timeout is 0.25 s. Driver faults or stale data return ERROR
to controller_manager and request a CAN quick-stop. The Python driver independently
watches command/feedback loss after command acquisition, including manager failure.
It does not automatically reset faults or disable motors on watchdog expiry.

A CAN quick-stop is a software command, not a safety-rated E-stop. With a severed
CAN connection software cannot guarantee delivery. Fault recovery requires
explicit operator recovery of the firmware and reconfiguration/reactivation of the
controller stack; stale commands are not resumed automatically.

The standard real launch explicitly sets `require_hardware_feedback=true`.
Legacy direct/dual/RViz driver entry points retain the opt-out node default for
compatibility and are **not this guarded ros2_control path**. Do not use them
concurrently with the real controller. Guarded mode rejects direct `/pos_cmd`.

## Tracking tolerances

The real arm JTC configs explicitly use measured state at activation/interpolation:

- `interpolate_from_desired_state: false`
- `set_last_command_interface_value_as_state_on_activation: false`
- path position tolerance: 0.10 rad per arm joint
- final position tolerance: 0.01 rad per arm joint
- final velocity tolerance: 0.02 rad/s, goal grace period: 2 s

These are initial operational limits, **not a millimetre accuracy certificate**.
Tune against measured tracking and the task requirements. Gripper position limits
are 0.01 m along the path and 0.002 m at goal; no velocity state is claimed for its
controller because the protocol does not report jaw velocity. joint7 is half the
jaw opening in metres; joint8 is passive/mimic. The driver converts to full jaw
opening once, independently of the legacy multiplier setting.

## Teach

`piper_teach_sync` uses fresh `/joint_states` and `/piper/teaching`, cancels active
FollowJointTrajectory goals and streams a measured hold during hand guidance.
The driver independently suppresses CAN target commands in teach mode. On exit,
command acquisition requires agreement with the current measured pose (0.02 rad
per arm joint, 0.002 m for the finger), preventing a jump to the previous target.
The standard real launch enables teach-sync; keep it enabled for hand guidance.

## Units and calibration

Angles use `math.pi/180000` and `180000/math.pi` at the CAN boundary, with rounding
only for integer wire units. Enabling the gripper preserves its measured opening.
After changing these conversions, independently validate existing TCP and hand-eye
calibrations; this change does not overwrite either calibration or motor zeros.

## Verification without a robot

From a sourced workspace:

```bash
PYTHONPATH=/root/py_global/lib/python3.12/site-packages:$PYTHONPATH python3 -m pytest \
  src/robots/piper_ros/src/piper/test/test_can_feedback.py \
  src/robots/piper_ros/src/piper/test/test_driver_commands.py
ROS_DOMAIN_ID=87 ./build/piper_hardware/test_piper_system
ROS_DOMAIN_ID=87 python3 src/robots/piper_ros/src/piper_hardware/test/trajectory_integration.py
ROS_DOMAIN_ID=87 PYTHONPATH=/root/py_global/lib/python3.12/site-packages:$PYTHONPATH \
  python3 src/robots/piper_ros/src/piper_hardware/test/driver_executor_integration.py
```

Integration scripts assert domain 87 and use synthetic measurements/fake SDKs;
they never open a CAN port. Tests cover measured startup, true tracking success,
path/goal rejection, missing/replayed feedback, driver fault, teach cancellation,
new-pose hold, and prompt quick-stop during an enable service request.
