package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.superstructure.deprecated.Elevator;

public class ElevatorCommand extends Command {

    private final Elevator elevator;

    public ElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
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