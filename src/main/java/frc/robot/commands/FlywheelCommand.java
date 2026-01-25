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
     * Checks if a line segment intersects with a rectangular region.
     * Uses parametric line representation and checks intersection with all 4 edges.
     *
     * @param p0 Start point of segment
     * @param p1 End point of segment
     * @param xMin Left boundary of rectangle
     * @param xMax Right boundary of rectangle
     * @param yMin Bottom boundary of rectangle
     * @param yMax Top boundary of rectangle
     * @return true if segment intersects rectangle
     */
    private boolean lineSegmentIntersectsRectangle(Translation2d p0, Translation2d p1,
                                                   double xMin, double xMax,
                                                   double yMin, double yMax) {
        // Check if either endpoint is inside the rectangle
        if (isPointInRectangle(p0, xMin, xMax, yMin, yMax) ||
            isPointInRectangle(p1, xMin, xMax, yMin, yMax)) {
            return true;
        }

        // Direction vector of our segment
        double dx = p1.getX() - p0.getX();
        double dy = p1.getY() - p0.getY();

        // Check intersection with all 4 edges
        // Top edge: from (xMin, yMax) to (xMax, yMax)
        if (segmentIntersectsSegment(p0.getX(), p0.getY(), dx, dy,
                                     xMin, yMax, xMax - xMin, 0)) {
            return true;
        }

        // Bottom edge: from (xMin, yMin) to (xMax, yMin)
        if (segmentIntersectsSegment(p0.getX(), p0.getY(), dx, dy,
                                     xMin, yMin, xMax - xMin, 0)) {
            return true;
        }

        // Left edge: from (xMin, yMin) to (xMin, yMax)
        if (segmentIntersectsSegment(p0.getX(), p0.getY(), dx, dy,
                                     xMin, yMin, 0, yMax - yMin)) {
            return true;
        }

        // Right edge: from (xMax, yMin) to (xMax, yMax)
        if (segmentIntersectsSegment(p0.getX(), p0.getY(), dx, dy,
                                     xMax, yMin, 0, yMax - yMin)) {
            return true;
        }

        return false;
    }

    /**
     * Checks if a point is inside a rectangle.
     */
    private boolean isPointInRectangle(Translation2d point, double xMin, double xMax,
                                       double yMin, double yMax) {
        return point.getX() >= xMin && point.getX() <= xMax &&
               point.getY() >= yMin && point.getY() <= yMax;
    }

    /**
     * Checks if two line segments intersect using parametric representation.
     * Segment 1: P(t) = (p0x, p0y) + t * (dx, dy), t in [0, 1]
     * Segment 2: Q(s) = (q0x, q0y) + s * (ex, ey), s in [0, 1]
     *
     * @return true if segments intersect
     */
    private boolean segmentIntersectsSegment(double p0x, double p0y, double dx, double dy,
                                             double q0x, double q0y, double ex, double ey) {
        // Calculate denominator (cross product of direction vectors)
        double denominator = ex * dy - ey * dx;

        // Parallel lines (or coincident) - no intersection for our purposes
        if (Math.abs(denominator) < 1e-10) {
            return false;
        }

        // Calculate t and s parameters
        double t = ((q0x - p0x) * ey - (q0y - p0y) * ex) / denominator;
        double s = ((q0x - p0x) * dy - (q0y - p0y) * dx) / denominator;

        // Check if intersection is within both segments
        return (t >= 0 && t <= 1 && s >= 0 && s <= 1);
    }

    /** State machine to determine robot's (future) position and how to act correspondingly */
    @Override
    public void execute() {
        // Get current position and velocity
        Translation2d currentPosition = drivetrain.getPose().getTranslation();
        Translation2d velocity = new Translation2d(
            drivetrain.getFieldVelocity().vxMetersPerSecond,
            drivetrain.getFieldVelocity().vyMetersPerSecond
        );

        // Calculate trajectory line segment
        Translation2d startPoint = currentPosition;
        Translation2d endPoint = currentPosition.plus(velocity.times(HOOD_DEACTUATION_TIME));

        // Define all 4 trench rectangular regions (based on FRC 2026 field specs)
        // Blue Left Trench (zones 3-4)
        boolean intersectsBlueLeft = lineSegmentIntersectsRectangle(
            startPoint, endPoint,
            1.067 - TRENCH_TOLERANCE, 3.369 + TRENCH_TOLERANCE,
            1.929 - TRENCH_TOLERANCE, 4.034 + TRENCH_TOLERANCE
        );

        // Blue Right Trench (zones 2-3, square)
        boolean intersectsBlueRight = lineSegmentIntersectsRectangle(
            startPoint, endPoint,
            4.626 - TRENCH_TOLERANCE, 7.291 + TRENCH_TOLERANCE,
            1.128 - TRENCH_TOLERANCE, 3.438 + TRENCH_TOLERANCE
        );

        // Red Left Trench (zones 2-3, square, diagonally opposite to blue right)
        boolean intersectsRedLeft = lineSegmentIntersectsRectangle(
            startPoint, endPoint,
            9.249 - TRENCH_TOLERANCE, 11.914 + TRENCH_TOLERANCE,
            4.631 - TRENCH_TOLERANCE, 6.941 + TRENCH_TOLERANCE
        );

        // Red Right Trench (zones 1-2)
        boolean intersectsRedRight = lineSegmentIntersectsRectangle(
            startPoint, endPoint,
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



