package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.groundIntake;

public class GroundIntakeCommand extends Command {
    private final groundIntake intake;
    private final ManualControls controls;
    
    public GroundIntakeCommand(groundIntake intake, ManualControls controls) {
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }
    
    @Override
    public void execute() {
        if (controls.intakeOut()) {
            // Left bumper: extend intake and spin roller
            intake.intakeOut();
        } else if (controls.intakeIn()) {
            // Right bumper: retract intake and stop roller
            intake.intakeIn();
        } else {
            // No input: stop everything
            intake.stop();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // This is a default command, so it never finishes
    }
}
