# TalonFX B-Line swerve chassis

This project is based on the official AdvantageKit 2026 TalonFX swerve template and uses B-Line
for autonomous path following. It includes high-frequency odometry, TalonFX/CANcoder hardware IO,
Pigeon 2 support, simulation, SysId commands, replay logging, and an AdvantageScope calibration
layout.

## Before deploying

1. Set the team number with `WPILib: Set Team Number` in VS Code.
2. Generate `src/main/java/frc/robot/generated/TunerConstants.java` for the real chassis with CTRE
   Tuner X, following the AdvantageKit TalonFX template instructions. The checked-in CAN IDs,
   encoder offsets, geometry, ratios, gains, and CAN bus name are example values only.
3. Tune the drive and steer loops, then tune the B-Line constraints and PID controllers in
   `BLineAutos.java`. Begin with the robot on blocks and conservative limits.
4. Import `AdvantageScope Swerve Calibration.json` into AdvantageScope for calibration and tuning.

## B-Line autonomous paths

`BLineAutos` connects B-Line to the drive pose, measured robot-relative chassis speeds, odometry
reset, and robot-relative velocity output. It also sends B-Line telemetry to AdvantageKit logs.

Create paths with the [B-Line editor](https://bline-web.pages.dev/) and export them to
`src/main/deploy/autos/paths`. Load one with `new Path("FileName")`, then add the command returned by
the shared `FollowPath.Builder` to the autonomous chooser. The included programmatic example is for
simulation and low-speed validation, not for competition use.

Build with:

```sh
./gradlew build
```

## Required calibration

No generic swerve constants are safe for a physical robot. Verify motor and encoder CAN IDs,
inversions, absolute encoder offsets, Pigeon ID, CAN bus, module locations, wheel radius, gearing,
current limits, and all feedback/feedforward gains before enabling.
