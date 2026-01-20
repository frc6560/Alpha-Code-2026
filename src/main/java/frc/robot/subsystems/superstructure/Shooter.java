package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);

    // Gear ratio: 24:18 (motor:flywheel)
    private static final double GEAR_RATIO = 24.0 / 18.0;

    // PID constants
    private static final double kP = 0.11;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kV = 0.12;
    private static final double kS = 0.0;

    private double targetRPM = 0;

    public Shooter() {
        leaderMotor = new TalonFX(20, "rio");
        followerMotor = new TalonFX(21, "rio");
        configureMotors();
    }

    private void configureMotors() {
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

        leaderMotor.getConfigurator().apply(config);

        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.StatorCurrentLimit = 80;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = 40;

        followerMotor.getConfigurator().apply(followerConfig);

        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        double rps = rpm / 60.0 * GEAR_RATIO;
        leaderMotor.setControl(velocityControl.withVelocity(rps));
    }

    public void setPercent(double percent) {
        leaderMotor.set(percent);
    }

    public void stop() {
        targetRPM = 0;
        leaderMotor.stopMotor();
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public double getVelocityRPM() {
        double motorRPS = leaderMotor.getVelocity().getValueAsDouble();
        return motorRPS * 60.0 / GEAR_RATIO;
    }

    public boolean atSetpoint() {
        return Math.abs(getVelocityRPM() - targetRPM) < 100;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
        SmartDashboard.putNumber("Flywheel/Actual RPM", getVelocityRPM());
        SmartDashboard.putNumber("Flywheel/Leader Output", leaderMotor.get());
        SmartDashboard.putNumber("Flywheel/Follower Output", followerMotor.get());
        SmartDashboard.putBoolean("Flywheel/At Setpoint", atSetpoint());
    }
}