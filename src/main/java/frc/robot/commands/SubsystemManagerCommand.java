package frc.robot.commands;

import frc.robot.ManualControls;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.swervedrive.*;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.BallGrabber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.SubsystemManager;
import frc.robot.subsystems.superstructure.SubsystemManager.WantedSuperState;
import frc.robot.subsystems.superstructure.Arm;

import java.util.Optional;

public class SubsystemManagerCommand extends Command{

    

    private final SwerveSubsystem swerveSubsystem;
    private final Elevator elevator;
    private final Arm arm;
    private final BallGrabber ballGrabber;

    private final ManualControls controls;
    private final SubsystemManager subsystemManager;


    public SubsystemManagerCommand(
        SwerveSubsystem swerve,
        Elevator elevator,
        Arm arm,
        BallGrabber ballGrabber,
        ManualControls controls,
        SubsystemManager subsystemManager
        ) {
        this.swerveSubsystem = swerve;
        this.elevator = elevator;
        this.arm = arm;
        this.ballGrabber = ballGrabber;
        this.controls = controls;
        this.subsystemManager = subsystemManager;

        addRequirements(subsystemManager);
    }
    


    public void initialize() {
        subsystemManager.stow();
        ballGrabber.stop();
        subsystemManager.setWantedState(WantedSuperState.Stow);
    }

    public void periodic() {
        
    }

    public void execute() {

        if (controls.goToStow()) { //A
            subsystemManager.setWantedState(WantedSuperState.Stow);
            subsystemManager.stow();

        } else if (controls.goToL2Ball()) { //X
            subsystemManager.setWantedState(WantedSuperState.RemoveBallL2);
            subsystemManager.removeBallL2();

        } else if (controls.goToL3Ball()) { //B
            subsystemManager.setWantedState(WantedSuperState.RemoveBallL3);
            subsystemManager.removeBallL3();

        } else if (controls.goToShootBall()) { //Y
            subsystemManager.setWantedState(WantedSuperState.ShootBall);
            subsystemManager.shootBall();

        } else if (controls.goToGroundBall()) { //Left Bumper (3 lines)
            subsystemManager.setWantedState(WantedSuperState.GroundBallIntake);
            subsystemManager.groundBallIntake();
        }

        if (controls.runOuttake() ) { //Left Trigger
            ballGrabber.runOuttake();
        } else if (controls.runIntake()){
            ballGrabber.runIntake();
        }
    }
}
