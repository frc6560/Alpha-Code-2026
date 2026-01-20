// package frc.robot.subsystems.superstructure;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import frc.robot.Constants.TurretConstants;

// import edu.wpi.first.math.controller.ArmFeedforward;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class Turret extends SubsystemBase {

// /*
//     private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TopLimitSwitchID);
//     private final DigitalInput botLimitSwitch = new DigitalInput(ElevatorConstants.BotLimitSwitchID);
// */  

//     private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID, "rio");
    
//     private final ArmFeedforward turretFeedForward = new ArmFeedforward(
//         TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
//     private double targetPos = 0;

//     public final TrapezoidProfile.Constraints turretConstraints =
//     new TrapezoidProfile.Constraints(TurretConstants.kMaxV, TurretConstants.kMaxA);

//     public final TrapezoidProfile turretTrapezoidProfile = new TrapezoidProfile(turretConstraints);

//     public TrapezoidProfile.State turGoalState = new TrapezoidProfile.State();
//     public TrapezoidProfile.State turSetpointState = new TrapezoidProfile.State();

//     private final PIDController pidController = new PIDController(
//         TurretConstants.kP,
//         TurretConstants.kI,
//         TurretConstants.kD
//     );

//     // NetworkTables
//     private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Turret");
//     private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
//     private final NetworkTableEntry ntPosition = ntTable.getEntry("Turret position");
//     private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

//     public Turret() {
//         // Configure TalonFX
//         TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

//         // PID and feedforward configuration
//         Slot0Configs slot0 = talonFXConfigs.Slot0;
//         slot0.kS = TurretConstants.kS;
//         slot0.kV = TurretConstants.kV;
//         slot0.kA = TurretConstants.kA;
//         slot0.kP = TurretConstants.kP;
//         slot0.kI = TurretConstants.kI;
//         slot0.kD = TurretConstants.kD;

//         // Set motor to brake mode
//         talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

//         turretMotor.getConfigurator().apply(talonFXConfigs.withSlot0(slot0));
    
//     }

//     public void setSetpoint(TrapezoidProfile.State nextSetpoint) { turSetpointState = nextSetpoint; }
    
//     public TrapezoidProfile.State getSetpoint() { return turSetpointState; }

//     /**
//      * Set the target angle for the turret using Motion Magic control
//      * @param goalDeg Target angle in degrees
//      */
//     public void setGoal(double goalDeg) {
//         goalDeg = ((goalDeg % 360) + 360) % 360;
//         turGoalState = new TrapezoidProfile.State((goalDeg/360), 0); 

//         setControl();
//         ntTargetPos.setDouble(goalDeg);
//     }

//     public TrapezoidProfile.State getGoal() { return turGoalState; }
//     public double getGoalValue() { return (turGoalState.position)*360;}

    
//     public double getTurretAngleDeg() {
//         // Get position from TalonFX internal encoder (in rotations)
//         double rotations = turretMotor.getPosition().getValueAsDouble();
//         // Convert rotations back to degrees (reverse the 81:1 gear ratio)
//         double angle = (rotations) * 360;
//         return angle;
//     }

//     /**
//      * Reset the internal encoder to zero at the current position
//      * Call this when the turret is in a known position (e.g., stow position)
//      */
//     public void resetEncoder() {
//         turretMotor.setPosition(0);
//     }

//     /**
//      * Reset encoder and set it to a specific angle
//      * @param currentAngleDeg The actual current angle of the turret in degrees
//      */
//     public void resetEncoderToAngle(double currentAngleDeg) {
//         double rotations = (currentAngleDeg / 360.0) * 25;
//         turretMotor.setPosition(rotations);
//     }

//     /**
//      * Get the current state of the turret based on its position
//      * @return Current turret state
//      */
    

//     public void stopMotor() {
//         turretMotor.set(0);
//     }


//     @Override
//     public void periodic() {
//         double angle = getTurretAngleDeg();
        
//         // Update NetworkTables
//         ntAngle.setDouble(angle);
//         ntPosition.setDouble(angle);

//         // Update SmartDashboard
//         SmartDashboard.putNumber("Current Turret Angle", angle);
        
//         setControl();
        
//     }

//     public void setControl() {
//         final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
//         TrapezoidProfile.State targetState = turretTrapezoidProfile.calculate(0.02, turSetpointState, turGoalState);

//         double targetDeg = (targetState.position) * 360.0;
//         double currentDeg = getTurretAngleDeg();

//         m_request.Position = targetState.position;
//         m_request.Velocity = targetState.velocity;
//         setSetpoint(targetState);
//         turretMotor.setControl(m_request);
//     }

// }