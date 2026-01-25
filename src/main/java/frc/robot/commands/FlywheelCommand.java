package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class FlywheelCommand extends Command{
    private final Flywheel flywheel;
    private final SwerveSubsystem drivetrain;

    private final double HOOD_DEACTUATION_TIME = 0.8; // in seconds
    private final double TRENCH_TOLERANCE = 0.5; // in meters, larger than trench boundary

    public FlywheelCommand(Flywheel flywheel, SwerveSubsystem drivetrain) {
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        addRequirements(flywheel);
    }


    /**
     * Checks if robot trajectory intersects with a rectangular trench region.
     * Uses parametric motion: position(t) = robotPos + velocity * t
     * For each boundary (constant x or y), solve for crossing time t and check:
     *   1. Is 0 <= t <= HOOD_DEACTUATION_TIME?
     *   2. Is the perpendicular coordinate within bounds at time t?
     *
     * @param robotPos Current robot position
     * @param velocity Robot velocity vector
     * @param xMin Left boundary of trench
     * @param xMax Right boundary of trench
     * @param yMin Bottom boundary of trench
     * @param yMax Top boundary of trench
     * @return true if trajectory intersects trench
     */
    private boolean trajectoryIntersectsTrench(Translation2d robotPos, Translation2d velocity,
                                               double xMin, double xMax,
                                               double yMin, double yMax) {
        double x0 = robotPos.getX();
        double y0 = robotPos.getY();
        double vx = velocity.getX();
        double vy = velocity.getY();

        // Check if robot starts inside the trench
        if (x0 >= xMin && x0 <= xMax && y0 >= yMin && y0 <= yMax) {
            return true;
        }

        // Check crossing LEFT boundary (x = xMin)
        // Solve: x0 + vx * t = xMin  =>  t = (xMin - x0) / vx
        if (Math.abs(vx) > 1e-10) {
            double t = (xMin - x0) / vx;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double y_at_crossing = y0 + vy * t;
                if (y_at_crossing >= yMin && y_at_crossing <= yMax) {
                    return true;
                }
            }
        }

        // Check crossing RIGHT boundary (x = xMax)
        // Solve: x0 + vx * t = xMax  =>  t = (xMax - x0) / vx
        if (Math.abs(vx) > 1e-10) {
            double t = (xMax - x0) / vx;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double y_at_crossing = y0 + vy * t;
                if (y_at_crossing >= yMin && y_at_crossing <= yMax) {
                    return true;
                }
            }
        }

        // Check crossing BOTTOM boundary (y = yMin)
        // Solve: y0 + vy * t = yMin  =>  t = (yMin - y0) / vy
        if (Math.abs(vy) > 1e-10) {
            double t = (yMin - y0) / vy;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double x_at_crossing = x0 + vx * t;
                if (x_at_crossing >= xMin && x_at_crossing <= xMax) {
                    return true;
                }
            }
        }

        // Check crossing TOP boundary (y = yMax)
        // Solve: y0 + vy * t = yMax  =>  t = (yMax - y0) / vy
        if (Math.abs(vy) > 1e-10) {
            double t = (yMax - y0) / vy;
            if (t >= 0 && t <= HOOD_DEACTUATION_TIME) {
                double x_at_crossing = x0 + vx * t;
                if (x_at_crossing >= xMin && x_at_crossing <= xMax) {
                    return true;
                }
            }
        }

        return false;
    }

    /** State machine to determine robot's (future) position and how to act correspondingly */
    @Override
    public void execute() {
        // Get current position and velocity
        Translation2d robotPos = drivetrain.getPose().getTranslation();
        Translation2d velocity = new Translation2d(
            drivetrain.getFieldVelocity().vxMetersPerSecond,
            drivetrain.getFieldVelocity().vyMetersPerSecond
        );

        // Check trajectory against all 4 trench regions (based on FRC 2026 field specs)
        // Blue Left Trench (zones 3-4)
        boolean intersectsBlueLeft = trajectoryIntersectsTrench(
            robotPos, velocity,
            1.067 - TRENCH_TOLERANCE, 3.369 + TRENCH_TOLERANCE,
            1.929 - TRENCH_TOLERANCE, 4.034 + TRENCH_TOLERANCE
        );

        // Blue Right Trench (zones 2-3, square)
        boolean intersectsBlueRight = trajectoryIntersectsTrench(
            robotPos, velocity,
            4.626 - TRENCH_TOLERANCE, 7.291 + TRENCH_TOLERANCE,
            1.128 - TRENCH_TOLERANCE, 3.438 + TRENCH_TOLERANCE
        );

        // Red Left Trench (zones 2-3, square, diagonally opposite to blue right)
        boolean intersectsRedLeft = trajectoryIntersectsTrench(
            robotPos, velocity,
            9.249 - TRENCH_TOLERANCE, 11.914 + TRENCH_TOLERANCE,
            4.631 - TRENCH_TOLERANCE, 6.941 + TRENCH_TOLERANCE
        );

        // Red Right Trench (zones 1-2)
        boolean intersectsRedRight = trajectoryIntersectsTrench(
            robotPos, velocity,
            13.171 - TRENCH_TOLERANCE, 15.473 + TRENCH_TOLERANCE,
            4.034 - TRENCH_TOLERANCE, 6.139 + TRENCH_TOLERANCE
        );

        // Stop flywheel if trajectory intersects ANY trench
        boolean intersectsAnyTrench = intersectsBlueLeft || intersectsBlueRight ||
                                       intersectsRedLeft || intersectsRedRight;

        if (intersectsAnyTrench) {
            flywheel.stop();
        } else {
            flywheel.setRPM(1000);
        }

        // Logging
        SmartDashboard.putNumber("Shooter/v_x", velocity.getX());
        SmartDashboard.putNumber("Shooter/v_y", velocity.getY());
        SmartDashboard.putBoolean("Shooter/intersects_trench", intersectsAnyTrench);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** But this should never run lol */
    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}



