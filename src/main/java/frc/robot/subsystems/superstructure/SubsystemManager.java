package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.ManualControls;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class SubsystemManager extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Elevator elevator;
    private final Arm arm;
    private final BallGrabber ballGrabber;

    private final ManualControls controls;


    public enum WantedSuperState {
        Stow,
        ScoreL1,
        ScoreL2,
        RemoveBallL2,
        RemoveBallL3,
        ShootBall,
        StationIntake,
        GroundCoralIntake,
        GroundBallIntake,
        GroundBallIntakePreBall,
        StowPostBall,
        RemoveBallL2PreBall,
        RemoveBallL3PreBall,
        Idle,
        StowWithoutBall,
        StowWithoutBallPreStow
    }

    public enum CurrentSuperState {
        Stow,
        ScoreL1,
        ScoreL2,
        RemoveBallL2,
        RemoveBallL3,
        ShootBall,
        StationIntake,
        GroundCoralIntake,
        GroundBallIntake,
        GroundBallIntakePreBall,
        StowPostBall,
        RemoveBallL2PreBall,
        RemoveBallL3PreBall,
        Idle,
        StowWithoutBall,
        StowWithoutBallPreStow
    }

    private WantedSuperState wantedSuperState = WantedSuperState.Stow;
    private CurrentSuperState currentSuperState = CurrentSuperState.Stow;
    private CurrentSuperState previousSuperState;


    public SubsystemManager(
        SwerveSubsystem swerve,
        Elevator elevator,
        Arm arm,
        BallGrabber ballGrabber,
        ManualControls controls
        ) {
        this.swerveSubsystem = swerve;
        this.elevator = elevator;
        this.arm = arm;
        this.ballGrabber = ballGrabber;
        this.controls = controls;
    }

    public void periodic() {
        currentSuperState = handStateTransitions();
        applyStates();

    }

    public CurrentSuperState handStateTransitions() {
        if (previousSuperState != currentSuperState) {
            switch (wantedSuperState) {
                default:
                    currentSuperState = CurrentSuperState.Stow;
                    break;
                case ScoreL1:
                    currentSuperState = CurrentSuperState.ScoreL1;
                    break;
                case ScoreL2:
                    currentSuperState = CurrentSuperState.ScoreL2;
                    break;
                case RemoveBallL2:
                    currentSuperState = CurrentSuperState.RemoveBallL2;
                    break;
                case RemoveBallL3:
                    currentSuperState = CurrentSuperState.RemoveBallL3;
                    break;
                case ShootBall:
                    currentSuperState = CurrentSuperState.ShootBall;
                    break;
                case StationIntake:
                    currentSuperState = CurrentSuperState.StationIntake;
                    break;
                case GroundCoralIntake:
                    currentSuperState = CurrentSuperState.GroundCoralIntake;
                    break;
                case GroundBallIntake:
                    currentSuperState = CurrentSuperState.GroundBallIntake;
                    break;
                case GroundBallIntakePreBall:
                    currentSuperState = CurrentSuperState.GroundBallIntakePreBall;
                    break;
                case StowPostBall:
                    currentSuperState = CurrentSuperState.StowPostBall;
                    break;
                case RemoveBallL2PreBall:
                    currentSuperState = CurrentSuperState.RemoveBallL2PreBall;
                    break;
                case RemoveBallL3PreBall:
                    currentSuperState = CurrentSuperState.RemoveBallL3PreBall;
                    break;
                case Idle:
                    currentSuperState = CurrentSuperState.Idle;
                    break;
                case StowWithoutBall:
                    currentSuperState = CurrentSuperState.Idle;
                    break;
                case StowWithoutBallPreStow:
                    currentSuperState = CurrentSuperState.Idle;
                    break;

            }
        }
        return currentSuperState;
        
    }

    public void applyStates() {
        switch (currentSuperState) {
            case Stow:
                stow();
                break;
            case ScoreL1:
                scoreL1();
                break;
            case ScoreL2:
                scoreL2();
                break;
            case RemoveBallL2:
                removeBallL2();
                break;
            case RemoveBallL3:
                removeBallL3();
                break;
            case ShootBall:
                shootBall();
                break;
            case StationIntake:
                stationIntake();
                break;
            case GroundCoralIntake:
                groundCoralIntake();
                break;
            case GroundBallIntake:
                groundBallIntake();
                break;
            case GroundBallIntakePreBall:
                groundBallIntakePreBall();
                break;
            case StowPostBall:
                stowPostBall();
                break;
            case RemoveBallL2PreBall:
                removeBallL2PreBall();
                break;
            case RemoveBallL3PreBall:
                removeBallL3PreBall();
                break;
            case Idle:
                idle();
                break;
            case StowWithoutBall:
                stowWithoutBall();
                break;
            case StowWithoutBallPreStow:
                stowWithoutBallPreStow();
                break;
        }
    }

    public void stow() {
        if (previousSuperState != CurrentSuperState.Stow) {
            
            wantedSuperState = WantedSuperState.Stow;
            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
            ballGrabber.stop();
        
        }

        else {
            ballGrabber.stop();
            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
            wantedSuperState = WantedSuperState.Stow;
        
        }
    }

    public void stowWithoutBall() {
        if (previousSuperState != CurrentSuperState.Stow) {
            
            wantedSuperState = WantedSuperState.Stow;
            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
        
        }

        else {
            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
            wantedSuperState = WantedSuperState.Stow;
        
        }
    }

    private void scoreL1() {

    }

    private void scoreL2() {

    }

    public void removeBallL2() {
        if (previousSuperState != CurrentSuperState.RemoveBallL2) {
            
            elevator.setGoal(ElevatorConstants.L2BALL);
            arm.setGoal(ArmConstants.REEF_POSITION_DEG_low);
            ballGrabber.runIntake();
            setWantedState(WantedSuperState.RemoveBallL2PreBall);
        }
    }

    public void removeBallL2PreBall() {
        if (ballGrabber.hasBall()) {
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
            ballGrabber.stop();
            setWantedState(WantedSuperState.Idle);

        } else {
            elevator.setGoal(ElevatorConstants.L2BALL);
            ballGrabber.runIntake();
            arm.setGoal(ArmConstants.REEF_POSITION_DEG_low);
            setWantedState(WantedSuperState.RemoveBallL2PreBall);
        }
        
    }

    public void removeBallL3() {
        if (previousSuperState != CurrentSuperState.RemoveBallL3) {
            elevator.setGoal(ElevatorConstants.L3BALL);
            arm.setGoal(ArmConstants.REEF_POSITION_DEG_high);
            ballGrabber.runIntake();
            setWantedState(WantedSuperState.RemoveBallL3PreBall);
        }
    }
    
    public void removeBallL3PreBall() {
        if (ballGrabber.hasBall()) {
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
            ballGrabber.stop();
            setWantedState(WantedSuperState.Idle);

        } else {
            ballGrabber.runIntake();
            arm.setGoal(ArmConstants.REEF_POSITION_DEG_low);
            elevator.setGoal(ElevatorConstants.L3BALL);
            
        }
    }

    public void shootBall() {

        if (previousSuperState != CurrentSuperState.ShootBall) {
            wantedSuperState = WantedSuperState.ShootBall;
            elevator.setGoal(ElevatorConstants.SHOOTBALL);
            arm.setGoal(ArmConstants.BARGE);   
            ballGrabber.stop();
        }
    }
    //add extra logic to auto stow post ball gone

    private void stationIntake() {

    }

    private void groundCoralIntake() {

    }

    public void groundBallIntake() {
        if (previousSuperState != CurrentSuperState.GroundBallIntake) {
            elevator.setGoal(ElevatorConstants.STOW);
            arm.setGoal(ArmConstants.PICKUP_POSITION_DEG);
            ballGrabber.runIntake();
            setWantedState(WantedSuperState.GroundBallIntakePreBall);
        }
    }
    
    public void groundBallIntakePreBall() {
        if (ballGrabber.hasBall()) {
            arm.setGoal(ArmConstants.STOW_POSITION_DEG);
            ballGrabber.startDelayedStop();
            setWantedState(WantedSuperState.StowPostBall);

        } else {
            ballGrabber.runIntake();
            arm.setGoal(ArmConstants.PICKUP_POSITION_DEG);
            elevator.setGoal(ElevatorConstants.STOW);
            
        }
    }

    public void stowWithoutBallPreStow() {
        if (previousSuperState != CurrentSuperState.StowPostBall) {
            setWantedState(WantedSuperState.StowWithoutBall);
        }
    }

    public void stowPostBall() {
        if (previousSuperState != CurrentSuperState.StowPostBall) {
            setWantedState(WantedSuperState.Stow);
        }
    }


    public void setWantedState(WantedSuperState wantedState) {
        wantedSuperState = wantedState;
    }
}
