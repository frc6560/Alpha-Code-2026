package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot.Constants.GroundIntakeConstants;

public class groundIntake extends SubsystemBase {

    /* ======================== Hardware ======================== */

    private final TalonFX extensionMotor;
    private final TalonFX rollerMotor;
    private final DigitalInput retractLimitSwitch;

    /* ======================== State ======================== */

    private boolean isExtended = false;
    private boolean springyModeActive = false;

    /* ======================== Simulation State ======================== */

    private double simExtensionPosition = 0.0; // 0 = retracted, 1 = extended
    private double simExtensionVelocity = 0.0;
    private double simExtensionCurrent = 0.0;

    private static final double SIM_MAX_VELOCITY = 10.0;
    private static final double SIM_ACCELERATION = 30.0;
    private static final double SIM_FRICTION = 0.8;
    private static final double SIM_LOAD_CURRENT = 15.0;
    private static final double SIM_OBSTACLE_CURRENT = 35.0;

    /* ======================== NetworkTables ======================== */

    private final NetworkTable ntTable =
            NetworkTableInstance.getDefault().getTable("GroundIntake");

    private final NetworkTableEntry ntExtensionSpeed =
            ntTable.getEntry("Extension Speed");
    private final NetworkTableEntry ntRollerSpeed =
            ntTable.getEntry("Roller Speed");
    private final NetworkTableEntry ntSpringyMode =
            ntTable.getEntry("Springy Mode Active");
    private final NetworkTableEntry ntExtensionCurrent =
            ntTable.getEntry("Extension Current");
    private final NetworkTableEntry ntLimitSwitch =
            ntTable.getEntry("Retract Limit");

    /* ======================== Mechanism2d ======================== */

    private final Mechanism2d mechanism = new Mechanism2d(3, 3);
    private final MechanismRoot2d intakeRoot;
    private final MechanismLigament2d intakeArm;
    private final MechanismLigament2d roller;

    /* ======================== Constructor ======================== */

    public groundIntake() {

        retractLimitSwitch =
                new DigitalInput(GroundIntakeConstants.RETRACT_LIMIT_SWITCH_ID);

        extensionMotor =
                new TalonFX(GroundIntakeConstants.EXTENSION_MOTOR_ID);

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extensionConfig.MotorOutput.Inverted =
                GroundIntakeConstants.EXTENSION_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs extensionLimits = new CurrentLimitsConfigs();
        extensionLimits.SupplyCurrentLimit =
                GroundIntakeConstants.EXTENSION_NORMAL_CURRENT_LIMIT;
        extensionLimits.SupplyCurrentLimitEnable = true;

        extensionConfig.CurrentLimits = extensionLimits;
        extensionMotor.getConfigurator().apply(extensionConfig);

        rollerMotor =
                new TalonFX(GroundIntakeConstants.ROLLER_MOTOR_ID);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted =
                GroundIntakeConstants.ROLLER_MOTOR_INVERTED
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs();
        rollerLimits.SupplyCurrentLimit =
                GroundIntakeConstants.ROLLER_CURRENT_LIMIT;
        rollerLimits.SupplyCurrentLimitEnable = true;

        rollerConfig.CurrentLimits = rollerLimits;
        rollerMotor.getConfigurator().apply(rollerConfig);

        /* ========== Mechanism2d - Linear Extension ========== */

        intakeRoot = mechanism.getRoot("IntakeRoot", 0.5, 0.5);

        // Extension arm - extends horizontally (0 degrees = right/forward)
        intakeArm =
                intakeRoot.append(
                        new MechanismLigament2d(
                                "Extension",
                                0.2, // Starts retracted (short length)
                                0,   // Horizontal
                                6,
                                new Color8Bit(Color.kOrange)));

        // Roller at the end of extension - spins to show intake action
        roller =
                intakeArm.append(
                        new MechanismLigament2d(
                                "Roller",
                                0.2,
                                90, // Perpendicular to extension
                                8,
                                new Color8Bit(Color.kGray)));

        SmartDashboard.putData("Ground Intake Mechanism", mechanism);
    }

    /* ======================== Commands ======================== */

    public void intakeOut() {
        isExtended = true;
        extensionMotor.set(GroundIntakeConstants.EXTENSION_OUT_SPEED);
        rollerMotor.set(GroundIntakeConstants.ROLLER_INTAKE_SPEED);
        updateSpringyMode(); // Check springy mode immediately when extending
    }

    public void intakeIn() {
        isExtended = false;
        springyModeActive = false;

        if (!isRetractLimitTriggered()) {
            extensionMotor.set(GroundIntakeConstants.EXTENSION_IN_SPEED);
            // Keep roller spinning while retracting to hold game piece
            rollerMotor.set(GroundIntakeConstants.ROLLER_INTAKE_SPEED);
        } else {
            // Fully stowed - stop everything
            extensionMotor.set(0);
            rollerMotor.stopMotor();
        }
        
        resetExtensionCurrentLimit();
    }

    public void stop() {
        isExtended = false;
        springyModeActive = false;
        extensionMotor.stopMotor();
        rollerMotor.stopMotor();
        resetExtensionCurrentLimit();
    }

    /* ======================== Springy Mode ======================== */

    private void updateSpringyMode() {

        double current =
                RobotBase.isSimulation()
                        ? simExtensionCurrent
                        : extensionMotor.getSupplyCurrent().getValueAsDouble();

        if (!springyModeActive) {
            // Enter springy mode when current spikes
            if (current >= GroundIntakeConstants.SPRINGY_CURRENT_THRESHOLD) {
                springyModeActive = true;
                setSpringyCurrentLimit();
            }
        } else {
            // Only exit springy mode when we're no longer extended
            // (on real robot, exit when retracting or stopped)
            if (!isExtended) {
                springyModeActive = false;
                resetExtensionCurrentLimit();
            }
        }
    }

    private void setSpringyCurrentLimit() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        extensionMotor.getConfigurator().refresh(config);

        config.CurrentLimits.SupplyCurrentLimit =
                GroundIntakeConstants.EXTENSION_SPRINGY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        extensionMotor.getConfigurator().apply(config);
    }

    private void resetExtensionCurrentLimit() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        extensionMotor.getConfigurator().refresh(config);

        config.CurrentLimits.SupplyCurrentLimit =
                GroundIntakeConstants.EXTENSION_NORMAL_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        extensionMotor.getConfigurator().apply(config);
    }

    /* ======================== Sensors ======================== */

    public boolean isRetractLimitTriggered() {
        if (RobotBase.isSimulation()) {
            // In simulation: triggered when position near zero AND moving toward/at zero velocity
            return simExtensionPosition <= 0.05;
        }
        return !retractLimitSwitch.get();
    }

    /* ======================== Periodic ======================== */

    @Override
    public void periodic() {

        /* ===== Simulation Physics ===== */
        if (RobotBase.isSimulation()) {

            // Get the commanded motor output (what the motor controller receives)
            double motorCommand = extensionMotor.get();
            
            // Calculate "actual" motor output based on physical constraints
            // In springy mode, current limiter reduces effective torque
            double effectiveMotorCommand = motorCommand;
            
            // Check if hitting physical limits
            boolean hittingExtensionLimit = simExtensionPosition >= 0.95 && motorCommand > 0;
            boolean hittingRetractionLimit = simExtensionPosition <= 0.05 && motorCommand < 0;
            
            if (hittingExtensionLimit || hittingRetractionLimit) {
                // When hitting obstacle, springy mode reduces effective power
                if (springyModeActive) {
                    effectiveMotorCommand *= 0.25; // Current limiter heavily reduces torque
                } else {
                    // Not yet in springy mode - full stall
                    effectiveMotorCommand = 0.0;
                }
            }

            // Physics simulation using effective motor command
            double dt = 0.02;
            double targetVelocity = effectiveMotorCommand * SIM_MAX_VELOCITY;
            double accelStep = SIM_ACCELERATION * dt;

            if (Math.abs(targetVelocity - simExtensionVelocity) < accelStep) {
                simExtensionVelocity = targetVelocity;
            } else {
                simExtensionVelocity +=
                        Math.signum(targetVelocity - simExtensionVelocity) * accelStep;
            }

            // Friction when coasting
            if (Math.abs(effectiveMotorCommand) < 0.01) {
                simExtensionVelocity *= (1.0 - SIM_FRICTION * dt);
            }

            simExtensionPosition += simExtensionVelocity * dt;
            simExtensionPosition = Math.max(0.0, Math.min(1.0, simExtensionPosition));

            // Current simulation - based on COMMANDED output, not effective
            // (Motor still tries to output full power, current limiter just caps current)
            simExtensionCurrent = Math.abs(motorCommand) * SIM_LOAD_CURRENT;

            // Massive current spike when hitting obstacle (before springy mode reduces it)
            if (hittingExtensionLimit || hittingRetractionLimit) {
                if (!springyModeActive) {
                    // Full current spike - motor stalling
                    simExtensionCurrent += SIM_OBSTACLE_CURRENT;
                } else {
                    // In springy mode - current limiter caps it
                    simExtensionCurrent = GroundIntakeConstants.EXTENSION_SPRINGY_CURRENT_LIMIT;
                }
            }
        }

        if (isExtended) {
            updateSpringyMode();
        }

        // When fully retracted (stowed), stop both motors
        if (!isExtended && isRetractLimitTriggered()) {
            extensionMotor.set(0);
            rollerMotor.stopMotor();
        }

        /* ===== Mechanism2d - Linear Extension ===== */

        // Change LENGTH to show extension (0.2m retracted, 1.0m extended)
        // Directly uses simulated position for accurate, real-time visualization
        double extensionLength =
                RobotBase.isSimulation()
                        ? 0.2 + (simExtensionPosition * 0.8) // 0.2 to 1.0 meters
                        : (isRetractLimitTriggered() ? 0.2 : 1.0);

        intakeArm.setLength(extensionLength);

        // Roller spins to show intake action - much faster for visibility
        // At 80% power (0.8 command), should spin very visibly fast
        if (Math.abs(rollerMotor.get()) > 0.01) {
            roller.setAngle(roller.getAngle() + rollerMotor.get() * 200); // Increased from 50 to 200
            roller.setColor(new Color8Bit(Color.kLimeGreen));
        } else {
            roller.setColor(new Color8Bit(Color.kGray));
        }

        // Extension color indicates state
        intakeArm.setColor(
                springyModeActive
                        ? new Color8Bit(Color.kRed)
                        : (isExtended
                            ? new Color8Bit(Color.kGreen)
                            : new Color8Bit(Color.kOrange)));

        /* ===== Telemetry ===== */

        ntExtensionSpeed.setDouble(extensionMotor.get());
        ntRollerSpeed.setDouble(rollerMotor.get());
        ntSpringyMode.setBoolean(springyModeActive);
        ntExtensionCurrent.setDouble(
                RobotBase.isSimulation()
                        ? simExtensionCurrent
                        : extensionMotor.getSupplyCurrent().getValueAsDouble());
        ntLimitSwitch.setBoolean(isRetractLimitTriggered());

        SmartDashboard.putNumber("Intake Sim Position", simExtensionPosition);
        SmartDashboard.putNumber("Intake Sim Velocity", simExtensionVelocity);
        SmartDashboard.putNumber("Intake Extension Current", ntExtensionCurrent.getDouble(0));
        SmartDashboard.putNumber("Intake Extension Length (m)", extensionLength);
        SmartDashboard.putNumber("Intake Extension Motor %", extensionMotor.get() * 100);
        SmartDashboard.putNumber("Intake Roller Motor %", rollerMotor.get() * 100);
    }
}
