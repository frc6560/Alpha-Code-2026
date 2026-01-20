package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase{
    private final TalonFX feedMotor;
    
    public Feeder() {
        this.feedMotor = new TalonFX(FeederConstants.MOTOR_ID, "rio");
    }
    public void feed() {
        feedMotor.set(4);
    }
    public void stop() {
        feedMotor.set(0);
    }
}
