package frc.robot.autonomous;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.Flywheel;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// TODOs: delete all commented out files in robot.java, and this file
public class AutoCommands {
    private SwerveSubsystem drivetrain;
    private Flywheel flywheel;
    private Feeder feeder;

    private AutoFactory autoFactory;

    public AutoCommands(SwerveSubsystem drivetrain, Flywheel flywheel, Feeder feeder) {
        this.drivetrain = drivetrain;
        this.flywheel = flywheel;
        this.feeder = feeder;

        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::followSegment,
            true,
            drivetrain
        );
    }

    /** These are functions for returning different autonomous routines. See AutoNames.java for more information. */

    /** These literally do nothing. As in, nothing. */
    public AutoRoutine getNoAuto(){
        final AutoRoutine IDLE = autoFactory.newRoutine("idle");
        return IDLE;
    }

    public Command spinUpShooter(){
        return Commands.runOnce(() -> flywheel.setRPM(ShooterConstants.FLYWHEEL_RPM), flywheel);
    }

    public Command shoot(){
        return Commands.run(() -> feeder.setRPM(FeederConstants.FEEDER_RPM), feeder)
            .withTimeout(3.0)
            .finallyDo((interrupted) -> {
                flywheel.setRPM(0);
                feeder.setRPM(0);
            });
    }

    /** Test auto on HP side. Should be comp level accuracy. */
    public AutoRoutine getrebuilt1(){
        AutoRoutine rebuilt1Routine = autoFactory.newRoutine("rebuilt1");
        
        AutoTrajectory p1 = rebuilt1Routine.trajectory("kianpth1dot1");
        AutoTrajectory p2 = rebuilt1Routine.trajectory("kianpath1dot2");
        AutoTrajectory p3 = rebuilt1Routine.trajectory("kianpath1dot3");
        AutoTrajectory p4 = rebuilt1Routine.trajectory("kianpath1dot4");
        AutoTrajectory p5 = rebuilt1Routine.trajectory("kianpath1dot5");

        p1.atTime("intake")
            .onTrue(
                Commands.idle()
            );

        rebuilt1Routine
            .active()
                .onTrue(
                    Commands.sequence(
                        p1.resetOdometry(),
                        p1.cmd(), // add an intake command after (or during) this.
                        p2.cmd()
                            .beforeStarting(p2.resetOdometry())
                            .andThen(Commands.waitSeconds(3)), // to simulate shooting
                        p3.cmd()
                            .beforeStarting(p3.resetOdometry()),
                        p4.cmd()
                            .beforeStarting(p4.resetOdometry())
                            .andThen(Commands.waitSeconds(3)),
                        p5.cmd()
                            .beforeStarting(p5.resetOdometry())
                    )
        );

        return rebuilt1Routine;
    }

    public AutoRoutine getrebuilt2(){
        AutoRoutine rebuilt2Routine = autoFactory.newRoutine("rebuilt2");
        
        AutoTrajectory p1 = rebuilt2Routine.trajectory("kianpth2dot1");
        AutoTrajectory p2 = rebuilt2Routine.trajectory("kianpath2dot2");
        AutoTrajectory p3 = rebuilt2Routine.trajectory("kianpath2dot3");
        AutoTrajectory p4 = rebuilt2Routine.trajectory("kianpath2dot4");

        p1.atTime("intake")
            .onTrue(
                Commands.idle()
            );
        
        p2.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );
        
        p4.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );

        rebuilt2Routine
            .active()
                .onTrue(
                    Commands.sequence(
                        p1.resetOdometry(),
                        p1.cmd(), // add an intake command after (or during) this.
                        p2.cmd()
                            .beforeStarting(p2.resetOdometry())
                            .andThen(Commands.waitSeconds(3)), // to simulate shooting
                        p3.cmd()
                            .beforeStarting(p3.resetOdometry()),
                        p4.cmd()
                            .beforeStarting(p4.resetOdometry())
                            .andThen(Commands.waitSeconds(3))
                    
                    )
        );

        return rebuilt2Routine;
    }
}





