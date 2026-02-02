package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.Shooter.ShotCalculator;

public class ShooterCommand extends Command{
    private final Shooter shooter;
    private final SwerveSubsystem drivetrain;
    private final ShotCalculator shotCalculator = new ShotCalculator();

    public ShooterCommand(Shooter shooter, SwerveSubsystem drivetrain) {
        this.shooter = shooter;
        this.drivetrain = drivetrain;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stop();
    }

    /** State machine to determine robot's (future) position and how to act correspondingly */
    @Override
        public void execute() {
            Pose2d robotPose = drivetrain.getPose();
            ChassisSpeeds robotVelocity = drivetrain.getFieldVelocity();

            shotCalculator.calculate(robotPose, robotVelocity);
            double flywheelSpeed = shotCalculator.getFlywheelRPM();
            shooter.setRPM(flywheelSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** But this should never run lol */
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}