package frc.robot.autonomous;

import java.util.Set;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utility.Enums.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// TODOs: delete all commented out files in robot.java, and this file
public class AutoFactory {
    private DriverStation.Alliance alliance;
    private SwerveSubsystem drivetrain;

    public AutoFactory(DriverStation.Alliance alliance, SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        if(alliance == null) {
            this.alliance = DriverStation.Alliance.Red;
        }
        else{
            this.alliance = alliance;
        }
    }

    /** These are functions for returning different autonomous routines. See AutoRoutines.java for more information. */

    private static final Command IDLE= Commands.idle();

    /** These literally do nothing. As in, nothing. */
    Pair<Pose2d, Command> getNoAutoLeft(){
        return Pair.of(null, IDLE);
    }

    Pair<Pose2d, Command> getNoAutoRight(){
        return Pair.of(null, IDLE);
    }

    /** These are four piece autos for various situations. All start on the left/right sides. */
    Pair<Pose2d, Command> getFourPieceBackRight(){
        return Pair.of(
            null,
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    } 

    Pair<Pose2d, Command> getFourPieceBackLeft(){
        return Pair.of(
            null,
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.BOTTOM_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    Pair<Pose2d, Command> getFourPieceLeft(){
        return Pair.of(
            null,
            IDLE
        );
    }

    Pair<Pose2d, Command> getFourPieceRight(){
        return Pair.of(
            null,
            IDLE
        );
    }

    Pair<Pose2d, Command> getThreePieceBackRight(){
        return Pair.of(
            null,
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    /** Scores one piece to the side, and two to the back */
    Pair<Pose2d, Command> getThreePieceBackLeft(){
        return Pair.of(
            null,
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    /** Scores 3 pieces in the center. */
    Pair<Pose2d, Command> getThreePieceCenter(){
        return Pair.of(
            null,
            Commands.defer(() -> Commands.sequence(
                drivetrain.getAutonomousCommand("shooter_1"),
                drivetrain.getAutonomousCommand("shooter_2"))
            , Set.of(drivetrain))
        );
    }

    Pair<Pose2d, Command> getTest(){
        return Pair.of(
            null,
            Commands.defer(() -> Commands.sequence(
                drivetrain.getAutonomousCommand("shooter_4"))
            , Set.of(drivetrain))
        );
    }

    public void updateAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }
}
