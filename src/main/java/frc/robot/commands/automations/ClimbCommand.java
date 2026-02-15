package frc.robot.commands.automations;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * PathPlanner-based climb command using on-the-fly pathfinding.
 * Much simpler and more reliable than manual velocity profiling!
 */
public class ClimbCommand extends SequentialCommandGroup {

    private final SwerveSubsystem drivetrain;
    private Pose2d targetPose;
    private Pose2d prescorePose;

    // Path constraints - match your robot's capabilities
    private static final double MAX_VELOCITY = 4.3; // m/s
    private static final double MAX_ACCEL = 9.0; // m/s^2
    private static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(540); // rad/s
    private static final double MAX_ANGULAR_ACCEL = Units.degreesToRadians(720); // rad/s^2

    public ClimbCommand(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;

        // Determine target based on alliance and starting position
        setTargets();

        addCommands(
            // Log what we're doing
            Commands.runOnce(() -> {
                SmartDashboard.putString("Climb/Target Pose", targetPose.toString());
                SmartDashboard.putString("Climb/Prescore Pose", prescorePose.toString());
                SmartDashboard.putString("Climb/Current Pose", drivetrain.getPose().toString());
                System.out.println("ClimbCommand starting:");
                System.out.println("  Current: " + drivetrain.getPose());
                System.out.println("  Prescore: " + prescorePose);
                System.out.println("  Final Target: " + targetPose);
            }),

            // Phase 1: Pathfind to prescore position
            getPathfindToPrescore(),

            // Phase 2: Pathfind to final climb position
            getPathfindToTarget()
        );

        addRequirements(drivetrain);
    }

    /** Phase 1: Pathfind to prescore position (1m back from target) */
    private Command getPathfindToPrescore() {
        PathConstraints constraints = new PathConstraints(
            MAX_VELOCITY,
            MAX_ACCEL,
            MAX_ANGULAR_VELOCITY,
            MAX_ANGULAR_ACCEL
        );

        return Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Climb/Phase 1 Active", true);
            SmartDashboard.putBoolean("Climb/Phase 2 Active", false);
        }).andThen(
            AutoBuilder.pathfindToPose(
                prescorePose,
                constraints,
                0.0  // goalEndVelocity - stop at prescore
            )
        ).andThen(
            Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Climb/Phase 1 Complete", true);
                System.out.println("Reached prescore position: " + drivetrain.getPose());
            })
        );
    }

    /** Phase 2: Pathfind to final climb position */
    private Command getPathfindToTarget() {
        // Use slower, more careful constraints for final approach
        PathConstraints constraints = new PathConstraints(
            MAX_VELOCITY * 0.5,  // Half speed for precision
            MAX_ACCEL * 0.7,
            MAX_ANGULAR_VELOCITY * 0.5,
            MAX_ANGULAR_ACCEL * 0.7
        );

        return Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Climb/Phase 1 Active", false);
            SmartDashboard.putBoolean("Climb/Phase 2 Active", true);
        }).andThen(
            AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0  // goalEndVelocity - stop at target
            )
        ).andThen(
            Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Climb/Phase 2 Complete", true);
                SmartDashboard.putBoolean("Climb/Complete", true);
                System.out.println("Reached final climb position: " + drivetrain.getPose());
            })
        );
    }

    /** Gets the prescore position 1 meter back from target */
    private Pose2d getPrescore(Pose2d targetPose) {
        DriverStation.Alliance alliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue);

        // Calculate prescore position 1 meter back from target
        double prescoreX;
        if (alliance.equals(DriverStation.Alliance.Blue)) {
            prescoreX = targetPose.getX() + 1.0; // Move back toward center (positive X)
        } else {
            prescoreX = targetPose.getX() - 1.0; // Move back toward center (negative X)
        }

        return new Pose2d(prescoreX, targetPose.getY(), targetPose.getRotation());
    }

    /** Sets the target pose based on alliance and starting Y position */
    private void setTargets() {
        DriverStation.Alliance alliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue);

        double initialY = drivetrain.getPose().getY();
        double yThreshold = 3.75;

        if (alliance.equals(DriverStation.Alliance.Blue)) {
            if (initialY > yThreshold) {
                targetPose = new Pose2d(1.5753228664398193, 4.183515548706055, new Rotation2d(0));
            } else {
                targetPose = new Pose2d(1.5753228664398193, 3.330711841583252, new Rotation2d(0));
            }
        } else {
            if (initialY > yThreshold) {
                targetPose = new Pose2d(14.976325035095215, 4.183515548706055, new Rotation2d(Math.PI));
            } else {
                targetPose = new Pose2d(14.977962493896484, 3.330711841583252, new Rotation2d(Math.PI));
            }
        }

        // Calculate prescore position
        prescorePose = getPrescore(targetPose);

        System.out.println("Climb targets set:");
        System.out.println("  Alliance: " + alliance);
        System.out.println("  Initial Y: " + initialY);
        System.out.println("  Target: " + targetPose);
        System.out.println("  Prescore: " + prescorePose);
    }
}
