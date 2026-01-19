package frc.robot.autonomous;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.superstructure.Feeder;
import frc.robot.subsystems.superstructure.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// TODOs: delete all commented out files in robot.java, and this file
public class AutoCommands {
    private SwerveSubsystem drivetrain;
    private Shooter shooter;
    private Feeder feeder;

    private AutoFactory autoFactory;

    public AutoCommands(SwerveSubsystem drivetrain, Shooter shooter, Feeder feeder) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
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
        return Commands.runOnce(() -> shooter.setRPM(ShooterConstants.FLYWHEEL_RPM), shooter);
    }

    public Command shoot(){
        return Commands.runOnce(() -> feeder.setRPM(FeederConstants.FEEDER_RPM), feeder)
            .withTimeout(3.0)
            .finallyDo((interrupted) -> {
                shooter.setRPM(0);
                feeder.setRPM(0);
            });
    }

    /** Test auto on HP side. Should be comp level accuracy. */
    public AutoRoutine getTestBump(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testBump");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpTrenchToShoot");
        AutoTrajectory bumpToShoot = testRoutine.trajectory("hpBumpToShoot");
        AutoTrajectory climb = testRoutine.trajectory("hpClimb");

        trenchToCenter.atTime("intake")
            .onTrue(
                Commands.idle()
            );
        
        trenchToShoot.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );
        
        bumpToShoot.atTime("shoot")
            .onTrue(
                spinUpShooter()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        trenchToCenter.resetOdometry(),
                        trenchToCenter.cmd(), // add an intake command after (or during) this.
                        trenchToShoot.cmd()
                            .beforeStarting(trenchToShoot.resetOdometry())
                            .andThen(shoot()), // to simulate shooting
                        trenchToCenter.cmd()
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        bumpToShoot.cmd()
                            .beforeStarting(bumpToShoot.resetOdometry())
                            .andThen(shoot()),
                        climb.cmd()
                            .beforeStarting(climb.resetOdometry())
                    )
        );

        return testRoutine;
    }

    public AutoRoutine getTestTrench(){
        AutoRoutine testRoutine = autoFactory.newRoutine("testTrench");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpTrenchToShoot");
        AutoTrajectory bumpToShoot = testRoutine.trajectory("hpBumpToShoot");
        AutoTrajectory trenchToClimb = testRoutine.trajectory("hpTrenchToClimb");

        trenchToCenter.atTime("intake")
            .onTrue(
                Commands.idle()
            );

        testRoutine
            .active()
                .onTrue(
                    Commands.sequence(
                        trenchToCenter.resetOdometry(),
                        trenchToCenter.cmd(), // add an intake command after (or during) this.
                        trenchToShoot.cmd()
                            .beforeStarting(trenchToShoot.resetOdometry())
                            .andThen(Commands.waitSeconds(3)), // to simulate shooting
                        trenchToCenter.cmd()
                            .beforeStarting(trenchToCenter.resetOdometry()),
                        trenchToClimb.cmd()
                            .beforeStarting(trenchToClimb.resetOdometry())
                    )
        );

        return testRoutine;
    }
}
