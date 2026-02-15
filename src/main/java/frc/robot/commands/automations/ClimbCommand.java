package frc.robot.commands.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that automatically aligns the robot to a target pose for climbing.
 * Uses simple PID control for X, Y, and rotation.
 */
public class ClimbCommand extends SequentialCommandGroup {

    // Poses
    private Pose2d targetPose;
    private Pose2d prescorePose;
    private double initialY;

    // PID Controllers (for final drive-in)
    private PIDController xController;
    private PIDController yController;
    private PIDController rotationController;

    // Pilot Drive Constants
    private static final double MAX_VELOCITY = 4.3; // m/s
    private static final double MAX_ACCEL = 9.0; // m/s^2
    private static final double SKID_ACCEL = 7.0; // m/s^2 (lateral acceleration limit)
    private static final double DT = 0.02; // 20ms loop time
    private static final double DISTANCE_THRESHOLD = 0.05; // meters
    private static final double TIMEOUT = 5.0; // seconds

    // Pilot Drive State
    private double currentVelocity = 0.0;
    private Timer pilotDriveTimer;

    // Subsystems
    private SwerveSubsystem drivetrain;


    /** Constructor for our climb command */
    public ClimbCommand(SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.initialY = drivetrain.getPose().getY();

        // Initialize PID controllers (tune these values as needed)
        this.xController = new PIDController(2.0, 0, 0);
        this.yController = new PIDController(2.0, 0, 0);
        this.rotationController = new PIDController(3.0, 0, 0);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Initialize timer
        this.pilotDriveTimer = new Timer();

        setTargets();

        super.addCommands(
            getDriveToPrescore(),
            getDriveInCommand()
        );
        super.addRequirements(drivetrain);
    }


    /** Drives to prescore position using pilot drive (angle bisector + autodrive) */
    public Command getDriveToPrescore() {
        return Commands.runOnce(() -> {
            currentVelocity = 0.0; // Reset velocity at start
            pilotDriveTimer.restart(); // Start timer
        }).andThen(Commands.run(() -> {
            prescorePose = getPrescore(targetPose);
            Pose2d currentPose = drivetrain.getPose();

            // Get positions
            Translation2d A = currentPose.getTranslation(); // Robot position
            Translation2d E = targetPose.getTranslation(); // Final target
            Translation2d F = prescorePose.getTranslation(); // Pre-target

            // Calculate vectors
            Translation2d AF = F.minus(A); // Vector from robot to pre-target
            Translation2d FE = E.minus(F); // Vector from pre-target to final target
            Translation2d AB = FE; // Direction of final approach (parallel to FE)

            // Calculate angle between AB and AF
            double angleAB = Math.atan2(AB.getY(), AB.getX());
            double angleAF = Math.atan2(AF.getY(), AF.getX());
            double alpha = angleAB - angleAF;

            // Normalize alpha to [-PI, PI]
            while (alpha > Math.PI) alpha -= 2 * Math.PI;
            while (alpha < -Math.PI) alpha += 2 * Math.PI;

            // Apply heuristic to alpha (modify curve shape)
            // Green heuristic (worlds version): moderate dampening
            double alphaModified = alpha * 0.5;

            // Calculate velocity direction: AF direction + modified alpha
            double velocityAngle = angleAF + alphaModified;

            // Calculate velocity magnitude using autodrive logic with kinematic equations
            double dx = A.getDistance(E); // Distance to final target

            // Limit 1: Stopping distance (using kinematic equation: vf² = vi² + 2*a*dx)
            // For stopping: 0² = v² - 2*a*dx → v = sqrt(2*a*dx)
            double maxVelStoppingDistance = Math.sqrt(2 * MAX_ACCEL * dx);

            // Limit 2: Forward acceleration (check if we can reach target velocity)
            // Using kinematic equation: v = v₀ + a*t
            double maxVelForwardAccel = currentVelocity + MAX_ACCEL * DT;

            // Check if we have enough distance to accelerate to maxVelForwardAccel and then stop
            // Distance needed to accelerate: d_accel = (v_target² - v_current²) / (2*a)
            // Distance needed to stop: d_stop = v_target² / (2*a)
            // Total distance needed: d_total = d_accel + d_stop
            if (maxVelForwardAccel > currentVelocity) {
                double distanceToAccelerate = (maxVelForwardAccel * maxVelForwardAccel - currentVelocity * currentVelocity) / (2 * MAX_ACCEL);
                double distanceToStop = (maxVelForwardAccel * maxVelForwardAccel) / (2 * MAX_ACCEL);
                double totalDistanceNeeded = distanceToAccelerate + distanceToStop;

                // If we don't have enough distance, cap the acceleration
                if (totalDistanceNeeded > dx) {
                    maxVelForwardAccel = currentVelocity; // Don't accelerate
                }
            }

            // Limit 3: Skid/lateral acceleration (turn radius)
            double turnRadius = calculateTurnRadius(A, F, E);
            double maxVelSkid = Double.MAX_VALUE;
            if (turnRadius > 0.01) { // Avoid division by zero
                // Centripetal acceleration: a = v² / r → v = sqrt(a * r)
                maxVelSkid = Math.sqrt(SKID_ACCEL * turnRadius);
            }

            // Take minimum of all limits
            double commandedVelocity = Math.min(MAX_VELOCITY,
                Math.min(maxVelStoppingDistance,
                    Math.min(maxVelForwardAccel, maxVelSkid)));

            // Update current velocity
            currentVelocity = commandedVelocity;

            // Calculate velocity components
            double vx = commandedVelocity * Math.cos(velocityAngle);
            double vy = commandedVelocity * Math.sin(velocityAngle);

            // Rotation control: align heading with velocity direction
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                velocityAngle
            );

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rotVel, currentPose.getRotation()));

            // Telemetry
            double distance = A.getDistance(F);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Prescore Translation Distance", distance);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Pilot Drive Velocity", commandedVelocity);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Pilot Drive Turn Radius", turnRadius);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Pilot Drive Alpha", Math.toDegrees(alpha));
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Pilot Drive Timer", pilotDriveTimer.get());
        }, drivetrain).until(() -> {
            double distance = drivetrain.getPose().getTranslation().getDistance(prescorePose.getTranslation());
            boolean atTarget = distance < DISTANCE_THRESHOLD;
            boolean timedOut = pilotDriveTimer.hasElapsed(TIMEOUT);

            if (timedOut) {
                System.out.println("Pilot drive timed out after " + TIMEOUT + " seconds");
            }

            return atTarget || timedOut;
        }));
    }

    /** Calculates turn radius for the current circular path */
    private double calculateTurnRadius(Translation2d A, Translation2d F, Translation2d E) {
        Translation2d AF = F.minus(A);
        Translation2d FE = E.minus(F);

        double angleAF = Math.atan2(AF.getY(), AF.getX());
        double angleFE = Math.atan2(FE.getY(), FE.getX());

        double deltaAngle = Math.abs(angleFE - angleAF);
        while (deltaAngle > Math.PI) deltaAngle = 2 * Math.PI - deltaAngle;

        // Approximate turn radius using chord length and angle
        double chordLength = AF.getNorm();
        if (Math.abs(deltaAngle) < 0.01) {
            return Double.MAX_VALUE; // Nearly straight
        }

        // R = chord / (2 * sin(angle/2))
        double radius = chordLength / (2 * Math.sin(deltaAngle / 2));
        return Math.abs(radius);
    }

    /** Drives to final climb position using braindead PID */
    public Command getDriveInCommand() {
        return Commands.run(() -> {
            Pose2d currentPose = drivetrain.getPose();

            double xVel = xController.calculate(currentPose.getX(), targetPose.getX());
            double yVel = yController.calculate(currentPose.getY(), targetPose.getY());
            double rotVel = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
            );

            drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, currentPose.getRotation()));

            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Climb Translation Distance", distance);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Climb Rotation Error", rotError);
        }, drivetrain).until(() -> {
            Pose2d currentPose = drivetrain.getPose();
            double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
            double rotError = Math.abs(currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians());
            return distance < 0.02 && rotError < 0.017;
        });
    }


    /** Gets the prescore position 1 meter back from target */
    public Pose2d getPrescore(Pose2d targetPose) {
        DriverStation.Alliance alliance;
        if (!DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.Alliance.Blue;
        } else {
            alliance = DriverStation.getAlliance().get();
        }

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
    public void setTargets() {
        DriverStation.Alliance alliance;
        if (!DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.Alliance.Blue;
        } else {
            alliance = DriverStation.getAlliance().get();
        }

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
    }
}