package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.SubsystemManager;
import frc.robot.subsystems.superstructure.SubsystemManager.WantedSuperState;

public class SubsystemManagerCommand extends Command{

    

    private final ManualControls controls;
    private final SubsystemManager subsystemManager;


    public SubsystemManagerCommand(
        ManualControls controls,
        SubsystemManager subsystemManager
        ) {
        this.controls = controls;
        this.subsystemManager = subsystemManager;

        addRequirements(subsystemManager);
    }
    


    public void initialize() {
        subsystemManager.stow();
        subsystemManager.setWantedState(WantedSuperState.Stow);
    }

    public void periodic() {
        
    }

    public void execute() {
        if (controls.goToStow()) { //A
            subsystemManager.setWantedState(WantedSuperState.Stow);
        } else {
            subsystemManager.setWantedState(WantedSuperState.Idle);
        }
    }
}
