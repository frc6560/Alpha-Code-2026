package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;

public class FlywheelCommand extends Command{
    private final Flywheel flywheel;
    private final SwerveSubsystem drivetrain;

    private final double HOOD_DEACTUATION_TIME = 0.8; // in seconds
    private final double TRENCH_TOLERANCE = 0.5; // in meters, larger than trench boundary
    private final double A_ALPHA = 0.2; // acceleration filter constant

    private double priorTime;
    private Translation2d v_previous;
    private Translation2d a;

    public FlywheelCommand(Flywheel flywheel, SwerveSubsystem drivetrain) {
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        priorTime = Timer.getFPGATimestamp();
        v_previous = new Translation2d(
            drivetrain.getFieldVelocity().vxMetersPerSecond,
            drivetrain.getFieldVelocity().vyMetersPerSecond
        );
        a = new Translation2d();
    }

    /** State machine to determine robot's (future) position and how to act correspondingly */
    @Override
        public void execute() {
        //dt 
        double currentTime = Timer.getFPGATimestamp();
        double dt = MathUtil.clamp(currentTime - priorTime, 0.005, 0.05);
        priorTime = currentTime;

        // numerically differentiate for acceleration. get velocity.
        Translation2d v = new Translation2d(
            drivetrain.getFieldVelocity().vxMetersPerSecond,
            drivetrain.getFieldVelocity().vyMetersPerSecond
        );
        Translation2d a_raw = v.minus(v_previous).div(dt);
        v_previous = v;

        // filter acceleration
        a = a.times(1 - A_ALPHA).plus(a_raw.times(A_ALPHA));

        // kiinematics!!
        Translation2d projectedPosition = drivetrain.getPose().getTranslation()
                                        .plus(v.times(HOOD_DEACTUATION_TIME))
                                        .plus(a.times(HOOD_DEACTUATION_TIME * HOOD_DEACTUATION_TIME / 2.0));
        
        Pose2d x0 = drivetrain.getSwerveDrive().getPose();

        if(projectedPosition.getDistance(Constants.FieldConstants.TRENCH_RED_LEFT) < TRENCH_TOLERANCE ||
           projectedPosition.getDistance(Constants.FieldConstants.TRENCH_RED_RIGHT) < TRENCH_TOLERANCE ||
           projectedPosition.getDistance(Constants.FieldConstants.TRENCH_BLUE_LEFT) < TRENCH_TOLERANCE ||
           projectedPosition.getDistance(Constants.FieldConstants.TRENCH_BLUE_RIGHT) < TRENCH_TOLERANCE||
           x0.getTranslation().getDistance(Constants.FieldConstants.TRENCH_RED_LEFT) < TRENCH_TOLERANCE ||
           x0.getTranslation().getDistance(Constants.FieldConstants.TRENCH_RED_RIGHT) < TRENCH_TOLERANCE ||
           x0.getTranslation().getDistance(Constants.FieldConstants.TRENCH_BLUE_LEFT) < TRENCH_TOLERANCE ||
           x0.getTranslation().getDistance(Constants.FieldConstants.TRENCH_BLUE_RIGHT) < TRENCH_TOLERANCE) {
            flywheel.stop();
        } else {
            flywheel.setRPM(1000);
        }
        // logging
        SmartDashboard.putNumber("Shooter/a_x", a.getX());
        SmartDashboard.putNumber("Shooter/a_y", a.getY());
        SmartDashboard.putNumber("Shooter/v_x", v.getX());
        SmartDashboard.putNumber("Shooter/v_y", v.getY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /** But this should never run lol */
    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }
}



