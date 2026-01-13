

package frc.robot.subsystems.superstructure;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class armm extends SubsystemBase {

    // --- Simulation state ---
    private double simAngleDeg = 90; 
    private double simTime = 0.0;
    private int currentTargetIndex = 0;
    private double stateStartTime = 0.0;
    private boolean isMoving = true;
    private final double[] targetPositions = {
        ArmConstants.STOW_POSITION_DEG, 
        ArmConstants.PICKUP_POSITION_DEG, 
        ArmConstants.REEF_POSITION_DEG_high,
        ArmConstants.REEF_POSITION_DEG_low, 
        ArmConstants.BARGE, 
        ArmConstants.PROCESSOR_DEG 
    };

    private final TalonFX armMotor;
    private final DutyCycleEncoder absoluteEncoder;

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Arm");
    private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
    private final NetworkTableEntry ntPosition = ntTable.getEntry("Arm position");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d root = mech.getRoot("ArmRoot", 30, 5);
    private final MechanismLigament2d armLigament =
        root.append(new MechanismLigament2d("Arm", 20, 90));

    public enum State {
        STOW,
        PICKUP,
        REEF_HIGH,
        REEF_LOW,
        GROUND,
        PROCESSOR,
        IN_MOTION
    }

    public armm() {
        armMotor = new TalonFX(ArmConstants.MOTOR_ID);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kS = ArmConstants.kS;
        slot0.kV = ArmConstants.kV;
        slot0.kA = ArmConstants.kA;
        slot0.kG = 0;
        slot0.kP = ArmConstants.kP;
        slot0.kI = ArmConstants.kI;
        slot0.kD = ArmConstants.kD;

        MotionMagicConfigs mm = talonFXConfigs.MotionMagic;
        mm.MotionMagicCruiseVelocity = ArmConstants.kMaxV;
        mm.MotionMagicAcceleration = ArmConstants.kMaxA;
        mm.MotionMagicJerk = 25.0;

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        armMotor.getConfigurator().apply(talonFXConfigs);

        // Absolute encoder
        absoluteEncoder = new DutyCycleEncoder(ArmConstants.ABS_ENCODER_DIO_PORT);

        // Mechanism2d setup
        armLigament.setLength(ArmConstants.ARM_LENGTH_METERS * 50);
        armLigament.setAngle(ArmConstants.STOW_POSITION_DEG);
        SmartDashboard.putData("ArmMech2d", mech);
    }

    public void setGoal(double goalDeg) {
      double rotations = (goalDeg / 360.0)*81;
  
      double gravityFF = ArmConstants.kG * Math.cos(Math.toRadians(getArmAngleDeg()));

      armMotor.setControl(
          new MotionMagicVoltage(gravityFF).withPosition(rotations)
      );
  }
  
    public double getArmAngleDeg() {
        if (RobotBase.isSimulation()) {
            return simAngleDeg;
        } else {
            double angle = absoluteEncoder.get() * 360.0;
            angle -= ArmConstants.ABS_ENCODER_OFFSET_DEG;
            if (ArmConstants.ABS_ENCODER_REVERSED) {
                angle = -angle;
            }
            return Math.IEEEremainder(angle, 360.0);
        }
    }

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
            return State.GROUND;
        } else if (Math.abs(angle - ArmConstants.PROCESSOR_DEG) < padding) {
            return State.PROCESSOR;
        } else if (Math.abs(angle - ArmConstants.REEF_POSITION_DEG_high) < padding){
            return State.REEF_HIGH;
        } else {
            return State.IN_MOTION;
        }
    }

    public void stopMotor(){
        armMotor.set(0);
      }

    @Override
    public void periodic() {
        // --- Simulation loop ---
        if (RobotBase.isSimulation()) {
            simTime += 0.02;

            double currentTarget = targetPositions[currentTargetIndex];
            double timeSinceStateStart = simTime - stateStartTime;

            if (isMoving) {
                double error = currentTarget - simAngleDeg;
                if (Math.abs(error) < 2.0) {
                    isMoving = false;
                    stateStartTime = simTime;
                    simAngleDeg = currentTarget;
                    System.out.println("Reached target: " + currentTarget + "°. Waiting...");
                } else {
                    simAngleDeg += error * 0.15;
                }
            } else {
                if (timeSinceStateStart >= 3.0) {
                    currentTargetIndex = (currentTargetIndex + 1) % targetPositions.length;
                    isMoving = true;
                    stateStartTime = simTime;
                    System.out.println("Moving to: " + targetPositions[currentTargetIndex] + "°");
                }
                simAngleDeg = currentTarget;
            }

            if ((int)(simTime * 50) % 50 == 0) {
                System.out.println("Sim angle: " + String.format("%.1f", simAngleDeg) +
                                   " | Target: " + String.format("%.1f", currentTarget) +
                                   " | Moving: " + isMoving +
                                   " | Time: " + String.format("%.1f", timeSinceStateStart) + "s");
            }
        }

        // --- Telemetry ---
        double angle = getArmAngleDeg();
        ntAngle.setDouble(angle);
        ntPosition.setDouble(angle);
        armLigament.setAngle(angle);

        SmartDashboard.putString("Arm State", getState().toString());
        SmartDashboard.putNumber("Current Angle", angle);
    }
}