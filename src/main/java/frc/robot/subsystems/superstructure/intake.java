package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.intakeConstants;

public class intake extends SubsystemBase {

	private final TalonFX intakeMotor;
	private final DigitalInput limitSwitch;
    private final Timer cooldownTimer = new Timer();
    private boolean inCooldown = false;

	public intake() {
		// Create hardware objects using constants
		this.intakeMotor = new TalonFX(intakeConstants.IntakeMotorID, "Canivore");
		this.limitSwitch = new DigitalInput(intakeConstants.IntakeLimitSwitchID);

		// Basic motor configuration: brake and reasonable inversion default
		TalonFXConfiguration fxConfig = new TalonFXConfiguration();
		fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		// Set inversion may need to flip
		fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		intakeMotor.getConfigurator().apply(fxConfig);

		ShuffleboardTab tab = Shuffleboard.getTab("Intake");
		tab.add(this);
	}

	@Override
	public void periodic() {
		//if the current gets to high just coast for a couple of secs to not fry the motor
		double current = getMotorCurrent();
		SmartDashboard.putNumber("Intake Current (A)", current);
		SmartDashboard.putBoolean("Intake In Cooldown", inCooldown);

		if (!inCooldown && current >= intakeConstants.MAX_CURRENT_AMPS) {
			// Enter cooldown
			enterCooldown();
		}

		if (inCooldown) {
			double elapsed = cooldownTimer.get();
			double remaining = Math.max(0.0, intakeConstants.COOLDOWN_SECONDS - elapsed);
			SmartDashboard.putNumber("Intake Cooldown Remaining (s)", remaining);
			if (elapsed >= intakeConstants.COOLDOWN_SECONDS) {
				exitCooldown();
			}
		}

	}

	// intake controls (renamed to avoid confusion with BallGrabber that doesent exist)
	public void runIntakeTwo() {
		if (inCooldown) {
			// Ignore requests while in cooldown
			return;
		}
		intakeMotor.set(intakeConstants.INTAKE_SPEED);
	}

	public void runOuttakeTwo() {
		if (inCooldown) {
			// Block while in cool beans
			return;
		}
		intakeMotor.set(intakeConstants.OUTTAKE_SPEED);
	}

	public void stop() {
		intakeMotor.stopMotor();
	}

	public double getMotorCurrent() {
		return intakeMotor.getSupplyCurrent().getValueAsDouble();
	}

	private void enterCooldown() {
		inCooldown = true;
		// set motor to coast and stop
		TalonFXConfiguration cfg = new TalonFXConfiguration();
		cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		intakeMotor.getConfigurator().apply(cfg);
		intakeMotor.stopMotor();
		cooldownTimer.reset();
		cooldownTimer.start();
	}

	private void exitCooldown() {
		inCooldown = false;
		// restore brake mode
		TalonFXConfiguration cfg = new TalonFXConfiguration();
		cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		intakeMotor.getConfigurator().apply(cfg);
		cooldownTimer.stop();
		SmartDashboard.putNumber("Intake Cooldown Remaining (s)", 0.0);
	}

	// Hardware accessors
	public TalonFX getMotor() {
		return intakeMotor;
	}
	//return value of limit switch
	public boolean getLimitSwitch() {
		return limitSwitch.get();
	}

}
