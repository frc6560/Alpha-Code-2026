package frc.robot.subsystems.superstructure.deprecated;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class Arm extends SubsystemBase {

    private final TalonFX armMotor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    // NetworkTables
    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");
    private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
    private final NetworkTableEntry ntPosition = ntTable.getEntry("Arm position");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

    // Mechanism2d visualization
    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d root = mech.getRoot("ArmRoot", 30, 5);
    private final MechanismLigament2d armLigament =
        root.append(new MechanismLigament2d("Arm", 20, 90));

    public enum State {
        STOW,
        PICKUP,
        REEF_LOW,
        REEF_HIGH,
        BARGE,
        PROCESSOR,
        IN_MOTION
    }

    public Arm() {
        armMotor = new TalonFX(ArmConstants.MOTOR_ID, "Canivore");

        // Configure TalonFX
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // PID and feedforward configuration
        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kS = ArmConstants.kS;
        slot0.kV = ArmConstants.kV;
        slot0.kA = ArmConstants.kA;
        slot0.kG = 0; // Gravity feedforward handled separately
        slot0.kP = ArmConstants.kP;
        slot0.kI = ArmConstants.kI;
        slot0.kD = ArmConstants.kD;

        // Motion Magic configuration
        MotionMagicConfigs mm = talonFXConfigs.MotionMagic;
        mm.MotionMagicCruiseVelocity = ArmConstants.kMaxV;
        mm.MotionMagicAcceleration = ArmConstants.kMaxA;
        mm.MotionMagicJerk = 25.0;

        // Set motor to brake mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Apply configuration
        armMotor.getConfigurator().apply(talonFXConfigs);

        // Initialize Mechanism2d visualization
        armLigament.setLength(ArmConstants.ARM_LENGTH_METERS * 50);
        armLigament.setAngle(ArmConstants.STOW_POSITION_DEG);
        SmartDashboard.putData("ArmMech2d", mech);
    }

    /**
     * Set the target angle for the arm using Motion Magic control
     * @param goalDeg Target angle in degrees
     */
    public void setGoal(double goalDeg) {
        // Convert degrees to motor rotations (assuming 81:1 gear ratio)

        //goalDeg = Math.max(Math.min(goalDeg, ArmConstants.MAX_ANGLE_DEG), ArmConstants.MIN_ANGLE_DEG);

        double rotations = (goalDeg / 360.0) * 81;
        
        // Calculate gravity feedforward based on current angle
        double gravityFF = ArmConstants.kG * Math.cos(Math.toRadians(getArmAngleDeg()));

        // Set target with Motion Magic and gravity compensation
        armMotor.setControl(
            motionMagicRequest.withPosition(rotations).withFeedForward(gravityFF)
        );
        
        // Update NetworkTables
        ntTargetPos.setDouble(goalDeg);
    }

    /**
     * Get current arm angle in degrees using TalonFX internal encoder
     * @return Current angle in degrees
     */
    public double getArmAngleDeg() {
        // Get position from TalonFX internal encoder (in rotations)
        double rotations = armMotor.getPosition().getValueAsDouble();
        // Convert rotations back to degrees (reverse the 81:1 gear ratio)
        double angle = (rotations / 81.0) * 360.0;
        return angle;
    }

    /**
     * Reset the internal encoder to zero at the current position
     * Call this when the arm is in a known position (e.g., stow position)
     */
    public void resetEncoder() {
        armMotor.setPosition(0);
    }

    /**
     * Reset encoder and set it to a specific angle
     * @param currentAngleDeg The actual current angle of the arm in degrees
     */
    public void resetEncoderToAngle(double currentAngleDeg) {
        double rotations = (currentAngleDeg / 360.0) * 81;
        armMotor.setPosition(rotations);
    }

    /**
     * Get the current state of the arm based on its position
     * @return Current arm state
     */
    public State getState() {
        double padding = 1.5; // Tolerance in degrees
        double angle = getArmAngleDeg();

        if (Math.abs(angle - ArmConstants.STOW_POSITION_DEG) < padding) {
            return State.STOW;
        } else if (Math.abs(angle - ArmConstants.PICKUP_POSITION_DEG) < padding) {
            return State.PICKUP;
        } else if (Math.abs(angle - ArmConstants.REEF_POSITION_DEG_low) < padding) {
            return State.REEF_LOW;
        } else if (Math.abs(angle - ArmConstants.BARGE) < padding) {
            return State.BARGE;
        } else if (Math.abs(angle - ArmConstants.PROCESSOR_DEG) < padding) {
            return State.PROCESSOR;
        } else if (Math.abs(angle - ArmConstants.REEF_POSITION_DEG_high) < padding){
            return State.REEF_HIGH;
        } else {
            return State.IN_MOTION;
        }
    }

    /**
     * Stop the arm motor
     */
    public void stopMotor() {
        armMotor.set(0);
    }

    /**
     * Check if the arm is at the target position
     * @param targetAngleDeg Target angle in degrees
     * @param tolerance Tolerance in degrees (default: 2.0)
     * @return True if arm is within tolerance of target
     */
    public boolean atTarget(double targetAngleDeg, double tolerance) {
        return Math.abs(getArmAngleDeg() - targetAngleDeg) < tolerance;
    }

    /**
     * Check if the arm is at the target position with default tolerance
     * @param targetAngleDeg Target angle in degrees
     * @return True if arm is within 2 degrees of target
     */
    public boolean atTarget(double targetAngleDeg) {
        return atTarget(targetAngleDeg, 2.0);
    }

    /**
     * Get the current motor velocity in degrees per second
     * @return Velocity in degrees per second
     */
    public double getVelocityDegPerSec() {
        double rotationsPerSec = armMotor.getVelocity().getValueAsDouble();
        return (rotationsPerSec / 81.0) * 360.0;
    }

    /**
     * Get motor current draw
     * @return Current in amps
     */
    public double getMotorCurrent() {
        return armMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        double angle = getArmAngleDeg();
        
        // Update NetworkTables
        ntAngle.setDouble(angle);
        ntPosition.setDouble(angle);
        
        // Update Mechanism2d visualization
        armLigament.setAngle(angle);

        // Update SmartDashboard
        SmartDashboard.putString("Arm State", getState().toString());
        SmartDashboard.putNumber("Current Angle", angle);
        SmartDashboard.putNumber("Arm Velocity (deg/s)", getVelocityDegPerSec());
        SmartDashboard.putNumber("Arm Current (A)", getMotorCurrent());
    }
}
