package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;
import frc.robot.subsystems.superstructure.GroundIntake;

public class GroundIntakeCommand extends Command {
    private final GroundIntake intake;
    private final ManualControls controls;
    
    public GroundIntakeCommand(GroundIntake intake, ManualControls controls) {
        this.intake = intake;
        this.controls = controls;
        addRequirements(intake);
    }
    
    @Override
    public void execute() {
        if (controls.intakeOut()) {
            // Right bumper pressed: extend intake and spin roller (actively intaking)
            intake.intakeOut();
        } else {
            // Right bumper released: retract intake and stop roller
            intake.intakeIn();
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