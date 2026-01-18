package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class Turret {
    private Pose2d fieldTarget = new Pose2d(1.0, 4.0, new Rotation2d(0));
    private final SwerveSubsystem drivebase;
    
    public Turret(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

    public SwerveSubsystem getDrivebase() {
        return drivebase;
    }

    public Pose2d getFieldTarget() {
        return fieldTarget;
    }

}
