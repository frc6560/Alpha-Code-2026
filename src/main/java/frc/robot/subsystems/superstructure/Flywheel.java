// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;

public class Flywheel extends SubsystemBase {

  //motor
  private final TalonFX flywheelMotor;

  //control 
  private final VelocityVoltage velocityControl;

  private final NetworkTable limelightTable; 

  //lookup table for distance to RPM
  private final InterpolatingDoubleTreeMap distanceToRPM; 

  //pose supplier
  private final PoseSupplier poseSupplier;

    // get pose from swerve subsystem
  public interface PoseSupplier {
   Pose2d getPose();
   }

  //state
  private double targetRPM = 0.0;



  /** Creates a new Flywheel. */
  public Flywheel(PoseSupplier poseSupplier) {
    this.poseSupplier = poseSupplier;

    // initialize limelight network table
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    //initialize motor
    flywheelMotor = new TalonFX(FlywheelConstants.FLYWHEEL_ID, "rio");
     configureMotor(); 

    //initialize control
    velocityControl = new VelocityVoltage(0.0).withSlot(0);

    //build lookuptable from constants
    distanceToRPM = new InterpolatingDoubleTreeMap(); 
    for (int i = 0; i < FlywheelConstants.DISTANCE_RPM_TABLE.length; i++) {
      distanceToRPM.put(
        FlywheelConstants.DISTANCE_RPM_TABLE[i][0], 
        FlywheelConstants.DISTANCE_RPM_TABLE[i][1]
      );
    }

  }

 

  private void configureMotor() {
    //motor configuration
    TalonFXConfiguration config = new TalonFXConfiguration();

    //pid config
    config.Slot0 = FlywheelConstants.FLYWHEEL_PID_CONFIG;
    config.Slot0.kP = FlywheelConstants.kP;
    config.Slot0.kI = FlywheelConstants.kI;
    config.Slot0.kD = FlywheelConstants.kD;
    config.Slot0.kV = FlywheelConstants.kV;
    //config.Slot0 = config.Slot0;

    //motor output 
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    //current limits
    config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    flywheelMotor.getConfigurator().apply(config);
  }
  
    private double getDistanceToTarget() {
    Pose2d robotPose = poseSupplier.getPose();
    Translation2d hubPose = getHubPose(); 

    return robotPose.getTranslation().getDistance(hubPose);
  }

    /**
   * Gets goal position based on alliance 
   * @return Goal position on field (meters)
   */
  private Translation2d getHubPose() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      return FlywheelConstants.HUB_RED_POSITION;
    }
    return FlywheelConstants.HUB_BLUE_POSITION;
  }

  public void runWithPose(){
    double distance = getDistanceToTarget();
    
    //get RPM from lookup table
    targetRPM = distanceToRPM.get(distance); 
    setRPM(targetRPM);
  }

  public void setIdle(){
    setRPM(FlywheelConstants.FLYWHEEL_IDLE_RPM);

  }
  public void setRPM(double rpm){
    targetRPM = rpm;
    
    //convert to motor velocity 
    double motorRPM = rpm * FlywheelConstants.FLYWHEEL_GEAR_RATIO; 
    double motorRPS = motorRPM / 60.0;

    //send command to motor
    flywheelMotor.setControl(velocityControl.withVelocity(motorRPS));
  }
  
  public void stop(){
    targetRPM = 0.0;
    flywheelMotor.stopMotor();
  }

  public double getCurrentRPM(){
    double motorRPS = flywheelMotor.getVelocity().getValueAsDouble(); 
    double motorRPM = motorRPS * 60.0;
    return motorRPM / FlywheelConstants.FLYWHEEL_GEAR_RATIO;
   
  }

  public double getTargetRPM(){
    return targetRPM;
  }

  public boolean atTargetRPM(){
    double currentRPM = getCurrentRPM();
    return Math.abs(currentRPM - targetRPM) < FlywheelConstants.FLYWHEEL_RPM_TOLERANCE;
  }

  public double getDistance(){
    return getDistanceToTarget();
  }

  // * Gets current robot pose (for debugging)
  //  */
  public Pose2d getRobotPose() {
    return poseSupplier.getPose();
  }

  public boolean hasTarget(){
    return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

// public void setRPMFromDistance (double distance){
  //   //get RPM from lookup table
  //   targetRPM = distanceToRPM.get(distance); 
  //   setRPM(targetRPM);
  
  // } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Telemetry for AdvantageScope and SmartDashboard
    SmartDashboard.putNumber("Flywheel/Current RPM", getCurrentRPM());
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRPM);
    SmartDashboard.putBoolean("Flywheel/At Target", atTargetRPM());
    SmartDashboard.putBoolean("Flywheel/Has Target", hasTarget());
    SmartDashboard.putNumber("Flywheel/Distance", getDistance());
    SmartDashboard.putNumber("Flywheel/Voltage", flywheelMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Flywheel/Current Draw", flywheelMotor.getSupplyCurrent().getValueAsDouble());
  }
}
