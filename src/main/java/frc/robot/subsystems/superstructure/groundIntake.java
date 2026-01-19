
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class groundIntake extends SubsystemBase {
    
    private final TalonFX extensionMotor;
    private final TalonFX rollerMotor;
    private final DigitalInput retractLimitSwitch;
    
    private boolean isExtended = false;
    private boolean springyModeActive = false;
    
    // NetworkTables for debugging
    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("GroundIntake");
    private final NetworkTableEntry ntExtensionSpeed = ntTable.getEntry("Extension Speed");
    private final NetworkTableEntry ntRollerSpeed = ntTable.getEntry("Roller Speed");
    private final NetworkTableEntry ntSpringyMode = ntTable.getEntry("Springy Mode Active");
    private final NetworkTableEntry ntExtensionCurrent = ntTable.getEntry("Extension Current");
    private final NetworkTableEntry ntLimitSwitch = ntTable.getEntry("Retract Limit");
    
    public groundIntake() {
        // Initialize limit switch (triggered when fully retracted)
        retractLimitSwitch = new DigitalInput(GroundIntakeConstants.RETRACT_LIMIT_SWITCH_ID);
        
        // Initialize extension motor
        extensionMotor = new TalonFX(GroundIntakeConstants.EXTENSION_MOTOR_ID);
        TalonFXConfiguration extensionConfig = new TalonFXConfiguration();
        extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        extensionConfig.MotorOutput.Inverted = GroundIntakeConstants.EXTENSION_MOTOR_INVERTED ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        // Set normal current limits for extension motor
        CurrentLimitsConfigs extensionCurrentLimits = new CurrentLimitsConfigs();
        extensionCurrentLimits.SupplyCurrentLimit = GroundIntakeConstants.EXTENSION_NORMAL_CURRENT_LIMIT;
        extensionCurrentLimits.SupplyCurrentLimitEnable = true;
        extensionConfig.CurrentLimits = extensionCurrentLimits;
        
        extensionMotor.getConfigurator().apply(extensionConfig);
        
        // Initialize roller motor
        rollerMotor = new TalonFX(GroundIntakeConstants.ROLLER_MOTOR_ID);
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.MotorOutput.Inverted = GroundIntakeConstants.ROLLER_MOTOR_INVERTED ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        
        // Set current limits for roller motor
        CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();
        rollerCurrentLimits.SupplyCurrentLimit = GroundIntakeConstants.ROLLER_CURRENT_LIMIT;
        rollerCurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits = rollerCurrentLimits;
        
        rollerMotor.getConfigurator().apply(rollerConfig);
    }
    
    /**
     * Extend the intake and spin the roller outward
     */
    public void intakeOut() {
        isExtended = true;
        extensionMotor.set(GroundIntakeConstants.EXTENSION_OUT_SPEED);
        rollerMotor.set(GroundIntakeConstants.ROLLER_INTAKE_SPEED);
        updateSpringyMode();
    }
    
    /**
     * Retract the intake and stop the roller
     * Respects limit switch to prevent over-retraction
     */
    public void intakeIn() {
        isExtended = false;
        springyModeActive = false;
        
        // Only retract if limit switch is NOT triggered (when triggered, intake is fully retracted)
        if (!isRetractLimitTriggered()) {
            extensionMotor.set(GroundIntakeConstants.EXTENSION_IN_SPEED);
        } else {
            // Already fully retracted, stop motor
            extensionMotor.stopMotor();
        }
        
        rollerMotor.stopMotor();
        resetExtensionCurrentLimit();
    }
    
    /**
     * Stop both motors
     */
    public void stop() {
        isExtended = false;
        springyModeActive = false;
        extensionMotor.stopMotor();
        rollerMotor.stopMotor();
        resetExtensionCurrentLimit();
    }
    
    /**
     * Update springy mode based on extension state
     * When fully extended, reduce current limit to allow "spring back"
     */
    private void updateSpringyMode() {
        if (isExtended) {
            double motorOutput = Math.abs(extensionMotor.get());
            
            // If motor is running near full speed (trying to extend), enable springy mode
            if (motorOutput >= GroundIntakeConstants.EXTENSION_THRESHOLD) {
                if (!springyModeActive) {
                    springyModeActive = true;
                    setSpringyCurrentLimit();
                }
            }
        }
    }
    
    /**
     * Set the extension motor to springy mode (reduced current limit)
     */
    private void setSpringyCurrentLimit() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        extensionMotor.getConfigurator().refresh(config);
        
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.SupplyCurrentLimit = GroundIntakeConstants.EXTENSION_SPRINGY_CURRENT_LIMIT;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;
        
        extensionMotor.getConfigurator().apply(config);
    }
    
    /**
     * Reset the extension motor to normal current limit
     */
    private void resetExtensionCurrentLimit() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        extensionMotor.getConfigurator().refresh(config);
        
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.SupplyCurrentLimit = GroundIntakeConstants.EXTENSION_NORMAL_CURRENT_LIMIT;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;
        
        extensionMotor.getConfigurator().apply(config);
    }
    
    /**
     * Check if the retract limit switch is triggered
     * @return true if intake is fully retracted (limit switch triggered)
     */
    public boolean isRetractLimitTriggered() {
        // DigitalInput.get() returns false when triggered (normally closed magnetic switch)
        // So we invert it: !get() means triggered
        return !retractLimitSwitch.get();
    }
    
    @Override
    public void periodic() {
        // Update springy mode if intake is extended
        if (isExtended) {
            updateSpringyMode();
        }
        
        // Safety: If retracting and hit limit switch, stop motor
        if (!isExtended && isRetractLimitTriggered()) {
            extensionMotor.stopMotor();
        }
        
        // Update NetworkTables for debugging
        ntExtensionSpeed.setDouble(extensionMotor.get());
        ntRollerSpeed.setDouble(rollerMotor.get());
        ntSpringyMode.setBoolean(springyModeActive);
        ntExtensionCurrent.setDouble(extensionMotor.getSupplyCurrent().getValueAsDouble());
        ntLimitSwitch.setBoolean(isRetractLimitTriggered());
        
        // SmartDashboard updates
        SmartDashboard.putBoolean("Intake Extended", isExtended);
        SmartDashboard.putBoolean("Intake Springy Mode", springyModeActive);
        SmartDashboard.putBoolean("Intake Fully Retracted", isRetractLimitTriggered());
        SmartDashboard.putNumber("Intake Extension Current", extensionMotor.getSupplyCurrent().getValueAsDouble());
    }
    
    // Getters for telemetry
    public boolean isExtended() {
        return isExtended;
    }
    
    public boolean isSpringyModeActive() {
        return springyModeActive;
    }
    
    public double getExtensionCurrent() {
        return extensionMotor.getSupplyCurrent().getValueAsDouble();
    }
    
    public double getRollerCurrent() {
        return rollerMotor.getSupplyCurrent().getValueAsDouble();
    }
}
