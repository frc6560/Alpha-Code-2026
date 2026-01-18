package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class feeder extends SubsystemBase {

    private final TalonFX feederMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

    private static final double GEAR_RATIO = 1.0;

    private static final double kP = 0.11;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    private static final double kS = 0.0;

    private double targetRPM = 0;

    // Auto increment variables
    private final Timer incrementTimer = new Timer();
    private boolean autoIncrementEnabled = false;

    public feeder() {
    feederMotor = new TalonFX(21, "Canivore");
    configureMotor();
    incrementTimer.start();
    autoIncrementEnabled = true;  // Start automatically
    targetRPM = 0;
}

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kV = kV;
        config.Slot0.kS = kS;

        feederMotor.getConfigurator().apply(config);
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0 * GEAR_RATIO;
        feederMotor.setControl(velocityControl.withVelocity(rps));
    }

    public void setPercent(double percent) {
        feederMotor.set(percent);
    }

    public void stop() {
        targetRPM = 0;
        feederMotor.stopMotor();
    }

    public void startAutoIncrement() {
        autoIncrementEnabled = true;
        targetRPM = 0;
        incrementTimer.reset();
    }

    public void stopAutoIncrement() {
        autoIncrementEnabled = false;
        stop();
    }

    public double getVelocityRPM() {
        double motorRPS = feederMotor.getVelocity().getValueAsDouble();
        return motorRPS * 60.0 / GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getVelocityRPM() - targetRPM) < 100;
    }

    @Override
    public void periodic() {
        // Auto increment RPM every 1 second
        if (autoIncrementEnabled && incrementTimer.hasElapsed(1.0)) {
            targetRPM += 50;
            setRPM(targetRPM);
            incrementTimer.reset();
        }

        SmartDashboard.putNumber("Feeder/Target RPM", targetRPM);
        SmartDashboard.putNumber("Feeder/Actual RPM", getVelocityRPM());
        SmartDashboard.putNumber("Feeder/Motor Output", feederMotor.get());
        SmartDashboard.putBoolean("Feeder/At Setpoint", atSetpoint());
        SmartDashboard.putBoolean("Feeder/Auto Increment Enabled", autoIncrementEnabled);
        SmartDashboard.putNumber("Feeder/Timer", incrementTimer.get());
    }
}