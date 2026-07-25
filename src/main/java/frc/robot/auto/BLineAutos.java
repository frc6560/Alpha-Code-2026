// Copyright (c) 2026
// Use of this source code is governed by a BSD license.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.DefaultGlobalConstraints;
import frc.robot.lib.BLine.Path.Waypoint;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Configures B-Line and owns the chassis autonomous routines. */
public final class BLineAutos {
  // Start conservatively. Tune these values from logs on the finished robot.
  private static final double MAX_LINEAR_SPEED_METERS_PER_SEC = 4.0;
  private static final double MAX_LINEAR_ACCEL_METERS_PER_SEC_SQ = 3.0;
  private static final double MAX_ANGULAR_SPEED_DEG_PER_SEC = 360.0;
  private static final double MAX_ANGULAR_ACCEL_DEG_PER_SEC_SQ = 720.0;
  private static final double END_TRANSLATION_TOLERANCE_METERS = 0.05;
  private static final double END_ROTATION_TOLERANCE_DEG = 2.0;
  private static final double INTERMEDIATE_HANDOFF_RADIUS_METERS = 0.30;

  private final FollowPath.Builder pathBuilder;

  public BLineAutos(Drive drive) {
    Path.setDefaultGlobalConstraints(
        new DefaultGlobalConstraints(
            Math.min(MAX_LINEAR_SPEED_METERS_PER_SEC, drive.getMaxLinearSpeedMetersPerSec()),
            MAX_LINEAR_ACCEL_METERS_PER_SEC_SQ,
            Math.min(
                MAX_ANGULAR_SPEED_DEG_PER_SEC, Math.toDegrees(drive.getMaxAngularSpeedRadPerSec())),
            MAX_ANGULAR_ACCEL_DEG_PER_SEC_SQ,
            END_TRANSLATION_TOLERANCE_METERS,
            END_ROTATION_TOLERANCE_DEG,
            INTERMEDIATE_HANDOFF_RADIUS_METERS));

    configureAdvantageKitLogging();

    pathBuilder =
        new FollowPath.Builder(
                drive,
                drive::getPose,
                drive::getChassisSpeeds,
                drive::runVelocity,
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withDefaultShouldFlip()
            .withPoseReset(drive::setPose);
  }

  /** A small path for simulation and initial low-speed validation. Replace with game paths. */
  public Command exampleAuto() {
    Path path =
        new Path(
            new Waypoint(new Pose2d(1.0, 1.0, Rotation2d.kZero)),
            new Waypoint(new Pose2d(2.0, 1.0, Rotation2d.kZero)),
            new Waypoint(new Pose2d(2.0, 2.0, Rotation2d.kCCW_90deg)));
    return pathBuilder.build(path).withName("B-Line Example");
  }

  private static void configureAdvantageKitLogging() {
    FollowPath.setPoseLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setTranslationListLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setDoubleLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
    FollowPath.setBooleanLoggingConsumer(
        value -> Logger.recordOutput(value.getFirst(), value.getSecond()));
  }
}
