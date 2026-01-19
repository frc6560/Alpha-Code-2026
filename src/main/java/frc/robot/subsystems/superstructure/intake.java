package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import frc.robot.Constants.intakeConstants;

public class intake extends SubsystemBase {

	private final TalonFX intakeMotor;
	private final DigitalInput limitSwitch;
    private final Timer cooldownTimer = new Timer();
    private boolean inCooldown = false;
	// Mechanism2d visualization for AdvantageScope / Shuffleboard
	private final Mechanism2d mechanism = new Mechanism2d(2, 2);
	private final MechanismRoot2d mechRoot = mechanism.getRoot("IntakeBase", 1.0, 1.0);
	private final MechanismLigament2d intakeWheel = mechRoot.append(new MechanismLigament2d("IntakeWheel", 1.0, 0));
	// simulated angle used only in Robot simulation because TalonFX position is not simulated
	private double simAngleDeg = 0.0;

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
		// Publish the Mechanism2d so AdvantageScope / Shuffleboard can display it
		SmartDashboard.putData("IntakeMechanism", mechanism);
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

		// Update visualization: rotate the intake wheel based on motor encoder (real robot)
		// or motor output in simulation (Phoenix encoders aren't simulated by default)
		double angleDeg;
		if (RobotBase.isSimulation()) {
			// In sim: compute output RPM from percent output and gearbox, then integrate
			double percent = intakeMotor.get();
			double motorRpm = percent * intakeConstants.MOTOR_FREE_RPM;
			// outputRpm = motorRpm * (driving/driven)
			double outputRpm = motorRpm * ((double) intakeConstants.GEAR_DRIVING / (double) intakeConstants.GEAR_DRIVEN);
			// degrees to advance this periodic: outputRpm (rev/min) -> rev/sec = outputRpm/60
			// deg/sec = (outputRpm/60)*360 ; deg per tick = deg/sec * dt
			double degPerTick = (outputRpm / 60.0) * 360.0 * 0.02; // dt ~= 20ms
			simAngleDeg += degPerTick;
			angleDeg = simAngleDeg % 360.0;
			if (angleDeg < 0) {
				angleDeg += 360.0;
			}
		} else {
			double rotations = intakeMotor.getPosition().getValueAsDouble();
			angleDeg = (rotations * 360.0) % 360.0;
			if (angleDeg < 0) {
				angleDeg += 360.0;
			}
		}

		intakeWheel.setAngle(angleDeg);
		// shorten the visual when in cooldown
		intakeWheel.setLength(inCooldown ? 0.6 : 1.0);

	}

	// intake controls (renamed to avoid confusion with BallGrabber that doesent exist)
	public void runIntakeTwo() {
		if (inCooldown) {
			// Ignore requests while in cooldown
			return;
		}
		// Set to a percent output that corresponds to the requested output RPM
		double percent = outputRpmToPercent(intakeConstants.TARGET_OUTPUT_RPM);
		// Preserve intake direction sign from existing INTAKE_SPEED
		percent = Math.signum(intakeConstants.INTAKE_SPEED) * Math.abs(percent);
		intakeMotor.set(percent);
	}

	public void runOuttakeTwo() {
		if (inCooldown) {
			// Block while in cool beans
			return;
		}
		double percent = outputRpmToPercent(intakeConstants.TARGET_OUTPUT_RPM);
		percent = Math.signum(intakeConstants.OUTTAKE_SPEED) * Math.abs(percent);
		intakeMotor.set(percent);
	}

	public void stop() {
		intakeMotor.stopMotor();
	}

	public double getMotorCurrent() {
		if (RobotBase.isSimulation()) {
			// Simulate current in sim so the cooldown can trigger: scale motor output to amps
			return Math.abs(intakeMotor.get()) * 60.0;
		}
		return intakeMotor.getSupplyCurrent().getValueAsDouble();
	}

	/**
	 * Convert desired output RPM (after gearbox) to motor percent output for the Falcon.
	 * Uses constants in Constants.intakeConstants: gear teeth and motor free RPM.
	 */
	private double outputRpmToPercent(double outputRpm) {
		// motor RPM required = outputRpm * (driven / driving)
		double motorRpm = outputRpm * ((double) intakeConstants.GEAR_DRIVEN / (double) intakeConstants.GEAR_DRIVING);
		double percent = motorRpm / intakeConstants.MOTOR_FREE_RPM;
		// Clamp to [-1,1]
		if (percent > 1.0) percent = 1.0;
		if (percent < -1.0) percent = -1.0;
		return percent;
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
