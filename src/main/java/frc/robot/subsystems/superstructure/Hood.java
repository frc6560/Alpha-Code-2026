package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.TurretConstants;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(HoodConstants.MOTOR_ID, "rio");
    
    private final ArmFeedforward hoodFeedForward = new ArmFeedforward(
        HoodConstants.kS, HoodConstants.kV, HoodConstants.kA);
    private double targetPos = 0;

    public final TrapezoidProfile.Constraints hoodConstraints =
    new TrapezoidProfile.Constraints(HoodConstants.kMaxV, HoodConstants.kMaxA);
    public final TrapezoidProfile hoodTrapezoidProfile = new TrapezoidProfile(hoodConstraints);

    public TrapezoidProfile.State hoodGoalState = new TrapezoidProfile.State();
    public TrapezoidProfile.State hoodSetpointState = new TrapezoidProfile.State();
    private final PIDController pidController = new PIDController(
        HoodConstants.kP,
        HoodConstants.kI,
        HoodConstants.kD
    );

    public Hood() {
        // Configure TalonFX
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // PID and feedforward configuration
        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kS = HoodConstants.kS;
        slot0.kV = HoodConstants.kV;
        slot0.kA = HoodConstants.kA;
        slot0.kP = HoodConstants.kP;
        slot0.kI = HoodConstants.kI;
        slot0.kD = HoodConstants.kD;

        // Set motor to brake mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hoodMotor.getConfigurator().apply(talonFXConfigs.withSlot0(slot0));
    
    }

    public void setSetpoint(TrapezoidProfile.State nextSetpoint) { hoodSetpointState = nextSetpoint; }
    
    public TrapezoidProfile.State getSetpoint() { return hoodSetpointState; }
    /**
     * Set the target angle for the hood using Motion Magic control
     * @param goalDeg Target angle in degrees
     */
    public void setGoal(double goalDeg) {
        goalDeg = ((goalDeg % 360) + 360) % 360;
        hoodGoalState = new TrapezoidProfile.State((goalDeg/360), 0); 

        setControl();
    }

    public TrapezoidProfile.State getGoal() { return hoodGoalState; }
    public double getGoalValue() { return (hoodGoalState.position/25)*360;}


    public double getHoodAngleDeg() {
        // Get position from TalonFX internal encoder (in rotations)
        double rotations = hoodMotor.getPosition().getValueAsDouble();
        // Convert rotations back to degrees (reverse the 81:1 gear ratio)
        double angle = (rotations) * 360;
        return angle;
    }

    /**
     * Reset the internal encoder to zero at the current position
     * Call this when the turret is in a known position (e.g., stow position)
     */
    public void resetEncoder() {
        hoodMotor.setPosition(0);
    }

    /**
     * Reset encoder and set it to a specific angle
     * @param currentAngleDeg The actual current angle of the hood in degrees
     */
    public void resetEncoderToAngle(double currentAngleDeg) {
        double rotations = (currentAngleDeg / 360.0) * 25;
        hoodMotor.setPosition(rotations);
    }

    /**
     * Get the current state of the hood based on its position
     * @return Current hood state
     */
    

    public void stopMotor() {
        hoodMotor.set(0);
    }


    @Override
    public void periodic() {
        double angle = getHoodAngleDeg();
        

        // Update SmartDashboard
        SmartDashboard.putNumber("Current Hood Angle", angle);
        
        setControl();
        
    }

    public void setControl() {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        TrapezoidProfile.State targetState = hoodTrapezoidProfile.calculate(0.02, hoodSetpointState, hoodGoalState);

        double targetDeg = (targetState.position) * 360.0;
        double currentDeg = getHoodAngleDeg();

        m_request.Position = targetState.position;
        m_request.Velocity = targetState.velocity;
        setSetpoint(targetState);
        hoodMotor.setControl(m_request);
    }

}