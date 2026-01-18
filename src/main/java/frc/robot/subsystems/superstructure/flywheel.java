package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class flywheel extends SubsystemBase {

    private final TalonFX flywheelMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

    // Gear ratio: 24:18 (motor:flywheel)
    private static final double GEAR_RATIO = 24.0 / 18.0;

    // PID constants - tune these
    private static final double kP = 0.11;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    private static final double kS = 0.0;

    private double targetRPM = 0;

    public flywheel() {
        flywheelMotor = new TalonFX(20, "Canivore"); // Adjust CAN ID
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;

        flywheelMotor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0 * GEAR_RATIO;
        flywheelMotor.setControl(velocityControl.withVelocity(rps));
    }

    public void setPercent(double percent) {
        flywheelMotor.set(percent);
    }

    public void stop() {
        targetRPM = 0;
        flywheelMotor.stopMotor();
    }

    public double getVelocityRPM() {
        double motorRPS = flywheelMotor.getVelocity().getValueAsDouble();
        return motorRPS * 60.0 / GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getVelocityRPM() - targetRPM) < 100;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/Actual RPM", getVelocityRPM());
        SmartDashboard.putNumber("Flywheel/Motor Output", flywheelMotor.get());
        SmartDashboard.putBoolean("Flywheel/At Setpoint", atSetpoint());
    }
}