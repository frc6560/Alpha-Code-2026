package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.Feeder;

public class FeederCommand extends Command {
    private final Feeder feeder;
    private final ManualControls controls;
    
    public FeederCommand(Feeder feeder, ManualControls controls) {
        this.feeder = feeder;
        this.controls = controls;
        addRequirements(feeder);
    }
    
    @Override
    public void execute() {
        if (controls.BallOut()) {
            feeder.feed();
        } else {
            feeder.stop();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // This is a default command, so it never finishes
    }
}