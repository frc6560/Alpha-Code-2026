package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.GroundIntakeConstants;

import frc.robot.subsystems.swervedrive.*;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.subsystems.superstructure.Hood;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.GroundIntake;
import frc.robot.ManualControls;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Snotm extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Flywheel flywheel;
    private final Turret turret;
    private final ShooterLUT shooterLUT;
    private final Hood hood;
    private final Feeder feeder;
    private final GroundIntake intake;

    private final ManualControls controls;
    private Pose2d fieldTarget;

    public Snotm(
        SwerveSubsystem swerve,
        Flywheel flywheel,
        Turret turret,
        ManualControls controls,
        ShooterLUT shooterLUT,
        Hood hood,
        Feeder feeder,
        GroundIntake intake
        ) {
        this.swerveSubsystem = swerve;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterLUT = shooterLUT;
        this.hood = hood;
        this.controls = controls;
        this.feeder = feeder;
        this.intake = intake;
    }

    public void ShootBall() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d robotPose = swerveSubsystem.getPose();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            if (robotPose.getX() < 4.03) {
                shootState();
            } else if (robotPose.getX() > 4.03 && robotPose.getX() < 16.45-4.03) {
                passState();
            } else {
                idleState();
            }
        } else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (robotPose.getX() > 16.54-4.03) {
                shootState();
            } else if (robotPose.getX() < 16.54-4.03 && robotPose.getX() > 4.03) {
                passState();
            } else {
                idleState();
            }
        } else {
            idleState();
        }
    }

    public void initialize() {
        fieldTarget = getHubPose(DriverStation.getAlliance());
    }

    Pose2d getHubPose(Optional<Alliance> alliance) {
        
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.HUB_RED_POSITION;
        }else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FieldConstants.HUB_BLUE_POSITION;
        } else{
        return FieldConstants.HUB_BLUE_POSITION;
        }
    }

    Pose2d getPassPose(Optional<Alliance> alliance) {
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.PASS_RED_POSITION;
        }else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FieldConstants.PASS_BLUE_POSITION;
        } else{
        return FieldConstants.PASS_BLUE_POSITION;
        }
    }

    public void shootState() {
        Pose2d robotPose = swerveSubsystem.getPose();
        Pose2d fieldTarget = getHubPose(DriverStation.getAlliance());
        BallOut(robotPose, fieldTarget);
    }

    public void passState() {
        Pose2d robotPose = swerveSubsystem.getPose();
        Pose2d fieldTarget = getPassPose(DriverStation.getAlliance());
        BallOut(robotPose, fieldTarget);
    }

    public void idleState() {
        flywheel.setRPM(Constants.FlywheelConstants.FLYWHEEL_IDLE_RPM);
        turret.setGoal(0);
        hood.setGoal(0);
    }



    public void BallOut(Pose2d robotPose, Pose2d fieldTarget) {
        ChassisSpeeds robotSpeeds = swerveSubsystem.getRobotVelocity();

        double rx = robotSpeeds.vxMetersPerSecond; // forward
        double ry = robotSpeeds.vyMetersPerSecond; // left

        double dx = robotPose.getX() - fieldTarget.getX();
        double dy = robotPose.getY() - fieldTarget.getY();
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        double launchAngle = shooterLUT.getAngle(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        double launchRPM = shooterLUT.getRPM(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));

        //IF YOU OVERSHOOT THEN RECALCULATE THE ANGLE BASED ON THE INVERSE TANGENT OF THE VELOCITY COMPONENTS
        if (launchRPM > FlywheelConstants.FLYWHEEL_MAX_RPM) {
            flywheel.setRPM(FlywheelConstants.FLYWHEEL_MAX_RPM);
        } else {
            flywheel.setRPM(launchRPM);
        }

        turret.setGoal(launchAngle);

        // turret.setGoal(turretShootAngle);
        // hood.setGoal(launchAngle);
    }

}