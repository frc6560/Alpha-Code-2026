// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.HoodConstants;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class Hood extends SubsystemBase {

  // Hardware
  private final TalonFX hoodMotor;
  private final CANcoder absoluteEncoder;

  // Mechanism2d visualization
private Mechanism2d hoodMech;
private MechanismRoot2d hoodRoot;
private MechanismLigament2d hoodArm;

  // Control
  private final PositionVoltage positionControl;

  // Trapezoidal motion profiling
  private final TrapezoidProfile.Constraints hoodConstraints;
  private final TrapezoidProfile hoodTrapezoidProfile;
  private TrapezoidProfile.State hoodGoalState;
  private TrapezoidProfile.State hoodSetpointState;

  // Lookup table for distance to angle
  private final InterpolatingDoubleTreeMap distanceToAngle;

  // Pose supplier
  private final PoseSupplier poseSupplier;

  // Target tracking
  private double targetAngle = 0.0;

  // Get pose from swerve subsystem
  public interface PoseSupplier {
    Pose2d getPose();
  }

  /** Creates a new Hood. */
  public Hood(PoseSupplier poseSupplier) {
    this.poseSupplier = poseSupplier;

    // Initialize hardware
    hoodMotor = new TalonFX(HoodConstants.HOOD_MOTOR_ID, "rio");
    absoluteEncoder = new CANcoder(HoodConstants.HOOD_MOTOR_ID + 1, "rio");

    // Position control request
    positionControl = new PositionVoltage(0.0).withSlot(0);

    // Initialize trapezoidal profile
    hoodConstraints = new TrapezoidProfile.Constraints(
      HoodConstants.kMaxV,
      HoodConstants.kMaxA
    );
    hoodTrapezoidProfile = new TrapezoidProfile(hoodConstraints);
    hoodGoalState = new TrapezoidProfile.State();
    hoodSetpointState = new TrapezoidProfile.State();

    // Configure hardware
    configureAbsoluteEncoder();
    configureMotor();



    hoodMech = new Mechanism2d(200, 200);

  // Add a root point for the hood at the center
  hoodRoot = hoodMech.getRoot("Hood Root", 100, 100);
  // Create ligaments representing the hood (length = 50, initial angle = 0)
  hoodArm = hoodRoot.append(
    new MechanismLigament2d("Hood Arm", 50, 0));
    hoodArm.setColor(new Color8Bit(255, 165, 0));

  // Push the Mechanism2D to SmartDashboard (so AdvantageScope can see it)
  SmartDashboard.putData("Hood Mechanism", hoodMech);

  



    // Initialize lookup table
    distanceToAngle = new InterpolatingDoubleTreeMap();
    for (int i = 0; i < HoodConstants.DISTANCE_ANGLE_TABLE.length; i++) {
      distanceToAngle.put(
        HoodConstants.DISTANCE_ANGLE_TABLE[i][0], 
        HoodConstants.DISTANCE_ANGLE_TABLE[i][1]
      );

      

    }
  }

  /**
   * Configures the absolute encoder (CANcoder)
   */
  private void configureAbsoluteEncoder() {
    CANcoderConfiguration config = new CANcoderConfiguration();
    
    // Set sensor direction (reverse if needed based on your setup)
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    // Apply offset to align zero position
    config.MagnetSensor.MagnetOffset = 0.0; // Will be set during calibration
    
    absoluteEncoder.getConfigurator().apply(config);
  }

  /**
   * Configures the hood motor
   */
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID and feedforward gains for position control
    Slot0Configs slot0 = config.Slot0;
    slot0.kS = HoodConstants.HOOD_kS;
    slot0.kV = HoodConstants.HOOD_kV;
    slot0.kA = HoodConstants.HOOD_kA;
    slot0.kP = HoodConstants.HOOD_kP;
    slot0.kI = HoodConstants.HOOD_kI;
    slot0.kD = HoodConstants.HOOD_kD;

    // Set neutral mode to brake
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current limits
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.HOOD_CURRENT_LIMIT > 0;

    // Apply configuration
    hoodMotor.getConfigurator().apply(config.withSlot0(slot0));
  }

  /**
   * Calculates distance from robot to target hub
   */
  private double getDistanceToTarget() {
    Pose2d robotPose = poseSupplier.getPose();
    Translation2d hubPose = getHubPose();

    return robotPose.getTranslation().getDistance(hubPose);
  }

  /**
   * Gets the hub position based on alliance color
   */
  private Translation2d getHubPose() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return FlywheelConstants.HUB_RED_POSITION;
    }
    return FlywheelConstants.HUB_BLUE_POSITION;
  }

  /**
   * Automatically sets hood angle based on current pose and distance to target
   */
  public void runWithPose() {
    double distance = getDistanceToTarget();
    targetAngle = distanceToAngle.get(distance);
    setGoal(targetAngle);
  }

  /**
   * Set the target angle for the hood using trapezoidal motion profiling
   * @param goalDeg Target angle in degrees
   */
  public void setGoal(double goalDeg) {
    // Normalize angle to 0-360
    goalDeg = ((goalDeg % 360) + 360) % 360;
    targetAngle = goalDeg;
    
    // Convert degrees to rotations for the profile
    hoodGoalState = new TrapezoidProfile.State(goalDeg / 360.0, 0);
  }

  /**
   * Get the current goal state
   */
  public TrapezoidProfile.State getGoal() {
    return hoodGoalState;
  }

  /**
   * Get the goal angle in degrees
   */
  public double getGoalValue() {
    return hoodGoalState.position * 360.0;
  }

  /**
   * Set the current setpoint state
   */
  public void setSetpoint(TrapezoidProfile.State nextSetpoint) {
    hoodSetpointState = nextSetpoint;
  }

  /**
   * Get the current setpoint state
   */
  public TrapezoidProfile.State getSetpoint() {
    return hoodSetpointState;
  }

  /**
   * Stop hood movement
   */
  public void stop() {
    hoodMotor.stopMotor();
  }

  /**
   * Returns current hood angle from motor encoder (degrees)
   */
  public double getCurrentAngle() {
    double motorRotations = hoodMotor.getPosition().getValueAsDouble();
    return motorRotations * 360.0 / HoodConstants.HOOD_GEAR_RATIO;
  }

  /**
   * Returns current hood angle from absolute encoder (degrees)
   * Useful for verification and debugging
   */
  public double getAbsoluteAngle() {
    double absolutePositionRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    return absolutePositionRotations * 360.0;
  }

  /**
   * Returns target angle (degrees)
   */
  public double getTargetAngle() {
    return targetAngle;
  }

  /**
   * Returns distance to target (for telemetry)
   */
  public double getDistance() {
    return getDistanceToTarget();
  }

  /**
   * Check if hood is at target position
   */
  public boolean atTarget(double tolerance) {
    return Math.abs(getCurrentAngle() - targetAngle) < tolerance;
  }

  /**
   * Reset the internal encoder to zero at the current position
   */
  public void resetEncoder() {
    hoodMotor.setPosition(0);
  }

  /**
   * Reset encoder and set it to a specific angle
   * @param currentAngleDeg The actual current angle of the hood in degrees
   */
  public void resetEncoderToAngle(double currentAngleDeg) {
    double rotations = (currentAngleDeg / 360.0) * HoodConstants.HOOD_GEAR_RATIO;
    hoodMotor.setPosition(rotations);
  }

  /**
   * Execute the trapezoidal motion profile control
   */
  private void setControl() {
    // Calculate the next state using trapezoidal profile
    TrapezoidProfile.State targetState = hoodTrapezoidProfile.calculate(
      0.02, // 20ms period
      hoodSetpointState,
      hoodGoalState
    );

    // Convert profile state (in rotations) to motor rotations accounting for gear ratio
    double targetRotations = targetState.position * HoodConstants.HOOD_GEAR_RATIO;
    double targetVelocity = targetState.velocity * HoodConstants.HOOD_GEAR_RATIO;

    // Update setpoint
    setSetpoint(targetState);

    // Apply position control with velocity feedforward
    positionControl.Position = targetRotations;
    positionControl.Velocity = targetVelocity;
    hoodMotor.setControl(positionControl);
  }

  @Override
  public void periodic() {
    // Read manual angle
  // double manualAngle = 20.0; 
  //     SmartDashboard.getNumber("Hood/Manual Angle", targetAngle);
  // manualAngle = Math.max(0.0, Math.min(70.0, manualAngle));
  // setGoal(manualAngle);

  double manualAngle = SmartDashboard.getNumber(
    "Hood/Manual Angle", targetAngle
);
  // Existing logic
  setControl();

  double currentAngle = getCurrentAngle();
  double absoluteAngle = getAbsoluteAngle();

  SmartDashboard.putNumber("Hood/Current Angle (Motor)", currentAngle);
  SmartDashboard.putNumber("Hood/Current Angle (Absolute)", absoluteAngle);

  // Update Mechanism2d
  // hoodArm.setAngle(getCurrentAngle());
  hoodArm.setAngle(30);


    // Update SmartDashboard
    SmartDashboard.putNumber("Hood/Current Angle (Motor)", getCurrentAngle());
    SmartDashboard.putNumber("Hood/Absolute Angle (Encoder)", getAbsoluteAngle());
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putNumber("Hood/Goal Angle", getGoalValue());
    SmartDashboard.putNumber("Hood/Distance to Target", getDistance());
    SmartDashboard.putNumber("Hood/Angle Error", targetAngle - getCurrentAngle());
    SmartDashboard.putBoolean("Hood/At Target", atTarget(1.0));

    // Motor debugging info
    SmartDashboard.putNumber("Hood/Motor Position (rotations)", hoodMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Velocity (rps)", hoodMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hood/Motor Voltage (V)", hoodMotor.getMotorVoltage().getValueAsDouble());

    // Profile state telemetry
    SmartDashboard.putNumber("Hood/Setpoint Position", hoodSetpointState.position * 360.0);
    SmartDashboard.putNumber("Hood/Setpoint Velocity", hoodSetpointState.velocity * 360.0);

    // Check encoder discrepancy
    double encoderDiscrepancy = Math.abs(currentAngle - absoluteAngle);
    SmartDashboard.putNumber("Hood/Encoder Discrepancy", encoderDiscrepancy);
    SmartDashboard.putBoolean("Hood/Needs Rehome", encoderDiscrepancy > 5.0);
  }
}