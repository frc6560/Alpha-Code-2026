package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private final TalonFX intakeMotor;
    
    public Intake() {
        this.intakeMotor = new TalonFX(IntakeConstants.MOTOR_ID, "rio");
    }
    public void intake() {
        intakeMotor.set(4);
    }
    public void stop() {
        intakeMotor.set(0);
    }
}
