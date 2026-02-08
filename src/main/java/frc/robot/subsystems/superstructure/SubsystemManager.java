package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SubsystemManager extends SubsystemBase {
    public enum WantedSuperState {
        Stow,
        Idle
    }

    public enum CurrentSuperState {
        Stow,
        Idle
    }

    private WantedSuperState wantedSuperState = WantedSuperState.Stow;
    private CurrentSuperState currentSuperState = CurrentSuperState.Stow;

    public SubsystemManager() {}

    @Override
    public void periodic() {
        currentSuperState = handStateTransitions();
        applyStates();
    }

    private CurrentSuperState handStateTransitions() {
        switch (wantedSuperState) {
            case Idle:
                return CurrentSuperState.Idle;
            case Stow:
            default:
                return CurrentSuperState.Stow;
        }
    }

    private void applyStates() {
        switch (currentSuperState) {
            case Idle:
                setIdle();
                break;
            case Stow:
            default:
                stow();
                break;
        }
    }

    public void stow() {
        wantedSuperState = WantedSuperState.Stow;
    }

    public void setIdle() {
        wantedSuperState = WantedSuperState.Idle;
    }

    public void setWantedState(WantedSuperState wantedState) {
        wantedSuperState = wantedState;
    }

    public WantedSuperState getWantedState() {
        return wantedSuperState;
    }

    public CurrentSuperState getCurrentState() {
        return currentSuperState;
    }
}
