package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.SotmConstants;
import frc.robot.Constants.OperatorConstants;

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

    private final ManualControls controls;

    

    public Sotm(
        SwerveSubsystem swerve,
        Flywheel flywheel,
        Turret turret,
        ManualControls controls,
        ShooterLUT shooterLUT
        ) {
        this.swerveSubsystem = swerve;
        this.flywheel = flywheel;
        this.turret = turret;
        this.shooterLUT = shooterLUT;
        this.controls = controls;
    }

    public void periodic() {
        Pose2d robotPose = turret.getDrivebase().getPose();
        Pose2d fieldTarget = turret.getFieldTarget();

        double dx = robotPose.getX() - fieldTarget.getX();
        double dy = robotPose.getY() - fieldTarget.getY();
        double robotHeadingDeg = robotPose.getRotation().getDegrees();

        double launchAngle = shooterLUT.getAngle(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
        double launchRPM = shooterLUT.getRPM(Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2)));
    }

    public void execute() {

    }

}