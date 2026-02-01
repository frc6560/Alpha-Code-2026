package frc.robot.commands.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;

import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.AutoAlignPath;
import frc.robot.utility.Setpoint;

import frc.robot.utility.Enums.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A command that automatically aligns the robot to a target pose, actuates the elevator and wrist, scores, and retracts.
 * This command is used for scoring at the reef on any level, from L1-L4.
 */
public class ClimbCommand extends SequentialCommandGroup {

    // Poses
    private Pose2d targetPose;

    // Paths
    private AutoAlignPath path;

    // Profiles
    private TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // The position is actually the error.
    private TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.Constraints translationConstraints;
    private TrapezoidProfile.Constraints rotationConstraint;
    private TrapezoidProfile translationProfile;
    private TrapezoidProfile rotationProfile;

    // Subsystems
    private SwerveSubsystem drivetrain;


    /** Constructor for our scoring command */
    public ClimbCommand(SwerveSubsystem drivetrain) {

        this.drivetrain = drivetrain;

        setTargets();

        super.addCommands(new ParallelCommandGroup(getDriveToPrescore()),
                                new ParallelCommandGroup(getDriveInCommand()));
        super.addRequirements(drivetrain);
    }


    /** Helper method for following a straight trajectory with a trapezoidal profile */
    public Command getFollowPath(AutoAlignPath path, double finalVelocity){
        final Command followPath = new FunctionalCommand(
            () -> {
            // Resets profiles and states
            translationConstraints = new Constraints(path.maxVelocity, path.maxAcceleration);
            rotationConstraint = new Constraints(path.maxAngularVelocity, path.maxAngularAcceleration);

            translationProfile = new TrapezoidProfile(translationConstraints);
            rotationProfile = new TrapezoidProfile(rotationConstraint);

            translationalState.position = path.getDisplacement().getNorm();
            translationalState.velocity = MathUtil.clamp(((-1) * (drivetrain.getFieldVelocity().vxMetersPerSecond * path.getDisplacement().getX() 
                                                                + drivetrain.getFieldVelocity().vyMetersPerSecond * path.getDisplacement().getY())/ translationalState.position),
                                                                -path.maxVelocity,
                                                                0);

            targetTranslationalState.velocity = finalVelocity;

            targetRotationalState.position = path.endPose.getRotation().getRadians();
            rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
            rotationalState.velocity = drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond;
        },
            () -> {
                // Move.
                Setpoint newSetpoint = getNextSetpoint(path);
                drivetrain.followSegment2(newSetpoint, targetPose);
                if(drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.02
                    && Math.abs(drivetrain.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.017
                ){
                    // Stop.
                    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
                }
            },
            (interrupted) -> {},
            () -> drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.02
            && Math.abs(drivetrain.getPose().getRotation().getRadians() - targetPose.getRotation().getRadians()) < 0.017
        );
        return followPath;

    }

    /** Drives close to our target pose during auto */
    public Command getDriveToPrescore(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            getPrescore(targetPose),
            DrivebaseConstants.kMaxAutoVelocity,
            DrivebaseConstants.kMaxAutoAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveToPrescore = getFollowPath(path, 2.1).until(
            () -> drivetrain.getPose().getTranslation().getDistance(getPrescore(targetPose).getTranslation()) < 0.2
        );
        return driveToPrescore;
    }

    /** Snaps to the reef pose */
    public Command getDriveInCommand(){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            targetPose, //originally just target pose
            DrivebaseConstants.kMaxAlignmentVelocity,
            DrivebaseConstants.kMaxAlignmentAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveIn = getFollowPath(path, 0);
        return driveIn;
    }

    /** Actuates superstructure to our desired level */


    /** Gets the prescore for a specific Pose2d */
    //todo convert m to foot
    public Pose2d getPrescore(Pose2d targetPose){
        return new Pose2d(
            targetPose.getX() + Units.metersToFeet(1) * Math.cos(targetPose.getRotation().getRadians()), 
            targetPose.getY() + Units.metersToFeet(1) * Math.sin(targetPose.getRotation().getRadians()), 
            targetPose.getRotation()
        );
    }

    /** Sets the target for the robot, including target pose, elevator height, and arm angle */
    public void setTargets(){
        DriverStation.Alliance alliance;
        if(!DriverStation.getAlliance().isPresent()){
            alliance = DriverStation.Alliance.Blue;
        }
        else alliance = DriverStation.getAlliance().get();



        targetPose = (alliance.equals(DriverStation.Alliance.Blue)) ? new Pose2d(): new Pose2d(); 
    }

    /** Transforms red alliance poses to blue by reflecting around the center point of the field*/
    public Pose2d applyAllianceTransform(Pose2d pose){
        return new Pose2d(
            pose.getX() - 2 * (pose.getX() - 8.75),
            pose.getY() - 2 * (pose.getY() - 4.0),
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );
    }

    /** Gets a setpoint for the robot PID to follow.
     * @return A Setpoint object representing the next target robot state on a certain auto align path.
     */
    public Setpoint getNextSetpoint(AutoAlignPath path){
        // trans
        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
        translationalState.position = translationSetpoint.position;
        translationalState.velocity = translationSetpoint.velocity;

        // rot wraparound calculations
        double rotationalPose = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
        double goalRotation = targetRotationalState.position;
        double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);

        targetRotationalState.position = rotationalPose + angularError;
        double setpointError = MathUtil.angleModulus(rotationalState.position - rotationalPose);
        rotationalState.position = rotationalPose + setpointError;
        // rot
        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
        rotationalState.position = rotationalSetpoint.position;
        rotationalState.velocity = rotationalSetpoint.velocity;

        // arc length parametrization for a line
        Translation2d interpolatedTranslation = path.endPose
            .getTranslation()
            .interpolate(path.startPose.getTranslation(), 
            translationSetpoint.position / path.getDisplacement().getNorm());

        return new Setpoint(
            interpolatedTranslation.getX(),
            interpolatedTranslation.getY(),
            rotationalState.position,
            path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, 
            path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
            rotationalState.velocity
        );
    }
}