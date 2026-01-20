package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.superstructure.Arm;
import frc.robot.subsystems.superstructure.Arm.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ManualControls;

public class ArmCommand extends Command {
    private final Arm arm;
    private final ManualControls controls;
    public State targetState;
    
    public ArmCommand(Arm arm, ManualControls controls) {
        this.arm = arm;
        this.controls = controls;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){ 

    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopMotor();
    }
}