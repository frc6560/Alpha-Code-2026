package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Elevator.WantedState;
import frc.robot.ManualControls;

public class ElevatorCommand extends Command {

    private final Elevator elevator;
    private final ManualControls controls;
    private WantedState targetState;

    public ElevatorCommand(Elevator elevator, ManualControls controls) {
        this.elevator = elevator;
        this.controls = controls;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.stopElev();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElev();
    }
}