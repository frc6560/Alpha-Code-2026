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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    final double t = 3.0; // time to predict ahead (get from CAD team)
    
    // x0 = initial pose
    Pose2d x0 = drivetrain.getSwerveDrive().getPose();
    
    // v = velocity magnitude (pythag of vx and vy)
    ChassisSpeeds robotVelocity = drivetrain.getSwerveDrive().getRobotVelocity();
    double vx = robotVelocity.vxMetersPerSecond;
    double vy = robotVelocity.vyMetersPerSecond;
    double v = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    
    // a = acceleration magnitude (pythag of ax and ay)
    Translation3d accel = drivetrain.getSwerveDrive().getAccel().get();
    double ax = accel.getX();
    double ay = accel.getY();
    double a = Math.sqrt(Math.pow(ax, 2) + Math.pow(ay, 2));
    
    // x_final = x0 + v*t + 0.5*a*t^2
    double xFinal = x0.getX() + (vx * t) + (0.5 * ax * Math.pow(t, 2));
    double yFinal = x0.getY() + (vy * t) + (0.5 * ay * Math.pow(t, 2));
    
    Translation2d futurePosition = new Translation2d(xFinal, yFinal);
    // Current pos check: shouldn't be in a circle radius
    // (you'll define the circle center and radius)
    Translation2d circleCenter = new Translation2d(2,3); //change later
    double radius = 0.0; // define this
    
    double distanceFromCenter = futurePosition.getDistance(circleCenter);
    double nowdistanceFromCenter = x0.getTranslation().getDistance(circleCenter);
    if (distanceFromCenter < radius || nowdistanceFromCenter < radius) {
        stop();
    }
}
    @Override
    public void execute(){ 


    }

    public void spinUpFlywheel(){
        flywheel.setRPM(-1500);
    }

    public void stop(){
        flywheel.stop();
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