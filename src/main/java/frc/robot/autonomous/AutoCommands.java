package frc.robot.autonomous;

import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// TODOs: delete all commented out files in robot.java, and this file
public class AutoCommands {
    private DriverStation.Alliance alliance;
    private SwerveSubsystem drivetrain;

    private final AutoFactory autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetOdometry,
            drivetrain::followSegment,
            true,
            drivetrain
        );

    public AutoCommands(DriverStation.Alliance alliance, SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        if(alliance == null) {
            this.alliance = DriverStation.Alliance.Red;
        }
        else{
            this.alliance = alliance;
        }

        // put AutoFactory bindings in here to intake, shoot, and so forwards
    }

    /** These are functions for returning different autonomous routines. See AutoRoutines.java for more information. */

    private final AutoRoutine IDLE = autoFactory.newRoutine("idle");

    /** These literally do nothing. As in, nothing. */
    Pair<Pose2d, AutoRoutine> getNoAuto(){
        return Pair.of(null, IDLE);
    }

    /** Test auto on HP side. Should be comp level accuracy. */
    public AutoRoutine getTest(){
        AutoRoutine testRoutine = autoFactory.newRoutine("test");
        
        AutoTrajectory trenchToCenter = testRoutine.trajectory("hpTrenchToCenter");
        AutoTrajectory trenchToShoot = testRoutine.trajectory("hpTrenchToShoot");
        AutoTrajectory bumpToShoot = testRoutine.trajectory("hpBumpToShoot");
        AutoTrajectory climb = testRoutine.trajectory("hpClimb");

        trenchToCenter.atTime("intake")
            .onTrue(
                null
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
                        bumpToShoot.cmd()
                            .beforeStarting(bumpToShoot.resetOdometry())
                            .andThen(Commands.waitSeconds(3)),
                        climb.cmd()
                            .beforeStarting(climb.resetOdometry())
                    )
        );

        return testRoutine;
    }

    public void updateAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }
}
