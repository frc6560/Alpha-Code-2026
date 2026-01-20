package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Arm.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.ManualControls;

public class FlywheelCommand extends Command {
    private final Flywheel flywheel;
    private final ManualControls controls;
    private SwerveSubsystem drivetrain;
    
    public FlywheelCommand(Flywheel flywheel, ManualControls controls, SwerveSubsystem drivetrain) {
        this.flywheel = flywheel;
        this.controls = controls;
        this.drivetrain = drivetrain;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        final double DEACTUATE_TIME = 3.0; // tune later
        Translation3d acceleration = drivetrain.getSwerveDrive().getAccel().get();
        double aSquaredMagnitude= Math.sqrt(Math.pow(acceleration.getX(), 2) + Math.pow(acceleration.getY(), 2));
        Translation2d accelerationParameter = new Translation2d(
            acceleration.getX(),
            acceleration.getY()
        );
        
        // get the velocity from the thing
        Translation2d velocity = new Translation2d(
            drivetrain.getSwerveDrive().getRobotVelocity().vxMetersPerSecond,
            drivetrain.getSwerveDrive().getRobotVelocity().vyMetersPerSecond
        );
        
    }

    @Override
    public void execute(){ 


    }

    public void spinUpFlywheel(){
        flywheel.setRPM(-1500);
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}