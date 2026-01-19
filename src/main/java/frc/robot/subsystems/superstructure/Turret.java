package frc.robot.subsystems.superstructure;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.FlywheelConstants;


public class Turret {
    
    private final SwerveSubsystem drivebase;

    public Turret(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

    public SwerveSubsystem getDrivebase() {
        return drivebase;
    }

    private Pose2d getHubPose() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FlywheelConstants.HUB_RED_POSITION;
        }else{
            return FlywheelConstants.HUB_BLUE_POSITION;
        }
    }

}
