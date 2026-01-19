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
import frc.robot.Constants.SotmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FlywheelConstants;

import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.superstructure.ShooterLUT;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.superstructure.Turret;
import frc.robot.ManualControls;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Sotm extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Flywheel flywheel;
    private final Turret turret;
    private final ShooterLUT shooterLUT;
    private final Hood hood;

    private final ManualControls controls;
    private Pose2d fieldTarget;

    public Sotm(
        SwerveSubsystem swerve,
        Flywheel flywheel,
        Turret turret,
        ManualControls controls,
        ShooterLUT shooterLUT,
        Hood hood
        ) {
        this.swerveSubsystem = swerve;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterLUT = shooterLUT;
        this.hood = hood;
        this.controls = controls;
    }

    public void periodic() {
        Pose2d robotPose = swerveSubsystem.getPose();
        Pose2d fieldTarget = getHubPose();

        ChassisSpeeds robotSpeeds = swerveSubsystem.getRobotVelocity();

        double rx = robotSpeeds.vxMetersPerSecond; // forward
        double ry = robotSpeeds.vyMetersPerSecond; // left

        double dx = robotPose.getX() - fieldTarget.getX();
        double dy = robotPose.getY() - fieldTarget.getY();
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        double launchAngle = shooterLUT.getAngle(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        double launchRPM = shooterLUT.getRPM(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        double launchVelocity = shooterLUT.getVelocity(launchRPM);

        double vk = launchVelocity * Math.sin(Math.toRadians(launchAngle));
        double vij = launchVelocity * Math.cos(Math.toRadians(launchAngle));
        double vi = vij * Math.cos(Math.toRadians(robotHeadingDeg));
        double vj = vij * Math.sin(Math.toRadians(robotHeadingDeg));

        ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

        double ri = fieldRelative.vxMetersPerSecond;
        double rj = fieldRelative.vyMetersPerSecond;

        double finalVi = vi-ri;
        double finalVj = vj-rj;
        double finalVk = vk;

        double finalVelocity = Math.sqrt(Math.pow(finalVi,2) + Math.pow(finalVj,2) + Math.pow(finalVk,2));
        double finalRPM = shooterLUT.getRPMForVelocity(finalVelocity);
        double turretShootAngle = Math.toDegrees(Math.atan2(finalVj, finalVi));

        //IF YOU OVERSHOOT THEN RECALCULATE THE ANGLE BASED ON THE INVERSE TANGENT OF THE VELOCITY COMPONENTS
        flywheel.setRPM(finalRPM);
        turret.setGoal(turretShootAngle);
        hood.setGoal(launchAngle);
    }

    public void initialize() {
        fieldTarget = getHubPose();
    }

    Pose2d getHubPose() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FlywheelConstants.HUB_RED_POSITION;
        }else if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return FlywheelConstants.HUB_BLUE_POSITION;
        } else{
        return FlywheelConstants.HUB_BLUE_POSITION;
        }
    }

}