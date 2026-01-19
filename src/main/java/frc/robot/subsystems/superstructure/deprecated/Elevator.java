package frc.robot.subsystems.superstructure.deprecated;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.util.sendable.SendableBuilder;  
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer; 

public class Elevator extends SubsystemBase {

    private final TalonFX ElevLeft = new TalonFX(ElevatorConstants.ElevLeftCanID, "Canivore");
    private final TalonFX ElevRight = new TalonFX(ElevatorConstants.ElevRightCanID, "Canivore");

    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TopLimitSwitchID);
    private final DigitalInput botLimitSwitch = new DigitalInput(ElevatorConstants.BotLimitSwitchID);

    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
            ElevatorConstants.kS, ElevatorConstants.kV, ElevatorConstants.kA);
    private double targetPos = 0;

    public final TrapezoidProfile.Constraints elevConstraints =
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxV, ElevatorConstants.kMaxA);

    public final TrapezoidProfile elevTrapezoidProfile = new TrapezoidProfile(elevConstraints);

    public TrapezoidProfile.State elevGoalState = new TrapezoidProfile.State();
    public TrapezoidProfile.State elevSetpointState = new TrapezoidProfile.State();

    private final PIDController pidController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
    );

    public enum WantedState {
        Stow,
        L2Ball,
        L3Ball,
        ShootBall,
        Idle,
    }

    private WantedState wantedState = WantedState.Idle;

    // Mechanism2d for SmartDashboard visualization
    private final Mechanism2d mechanism = new Mechanism2d(2, 5);
    private final MechanismRoot2d root = mechanism.getRoot("ElevatorBase", 1.0, 0);
    private final MechanismLigament2d elevatorLift =
            root.append(new MechanismLigament2d("ElevatorLift", 0.0, 90));

    // Shuffleboard tab for numeric + boolean properties
    private final ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    double startTime = 0;

    public Elevator() {
        // Keep mechanism in SmartDashboard

        // TalonFX motor configuration
        TalonFXConfiguration configLeft = new TalonFXConfiguration();
        TalonFXConfiguration configRight = new TalonFXConfiguration();
        configLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configRight.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs elevatorPID = new Slot0Configs();
        
        elevatorPID.kP = ElevatorConstants.kP;
        elevatorPID.kI = ElevatorConstants.kI;
        elevatorPID.kD = ElevatorConstants.kD;
        elevatorPID.kG = ElevatorConstants.kG;

        ElevLeft.getConfigurator().apply(configLeft.withSlot0(elevatorPID));
        ElevRight.getConfigurator().apply(configRight.withSlot0(elevatorPID));
        

        // Register this subsystem in Shuffleboard for AdvantageScope to see Sendable properties
        tab.add(this);

    }

    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {

            double elevatorHeight = getElevatorHeight();

            // Visualization
            double currentTime = Timer.getFPGATimestamp();
            double elapsedTime = currentTime - startTime;

            if (elapsedTime >= 3) {
                setGoal(ElevatorConstants.ElevState.SHOOTBALL.getValue());
            }

            if (elapsedTime >= 12) {
                setGoal(ElevatorConstants.ElevState.L2BALL.getValue());
            }

            if (elapsedTime >= 18) {
                setGoal(ElevatorConstants.ElevState.L3BALL.getValue());
                startTime = Timer.getFPGATimestamp();
            }

            SmartDashboard.putData("ElevatorMechanism", mechanism);
            
            setControl();

            
            

        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");

        // Numeric properties
        builder.addDoubleProperty("Height", this::getElevatorHeight, null);
        builder.addDoubleProperty("Setpoint", () -> elevSetpointState.position, null);
        builder.addDoubleProperty("Goal", () -> elevGoalState.position, null);

        // Boolean buttons to change WantedState
        builder.addBooleanProperty("GoToStow", () -> false, val -> setWantedState(WantedState.Stow));
        builder.addBooleanProperty("GoToL2Ball", () -> false, val -> setWantedState(WantedState.L2Ball));
        builder.addBooleanProperty("GoToL3Ball", () -> false, val -> setWantedState(WantedState.L3Ball));
        builder.addBooleanProperty("GoToShootBall", () -> false, val -> setWantedState(WantedState.ShootBall));
    }

    // Motor getters
    public TalonFX getElevLeft() { return ElevLeft; }
    public TalonFX getElevRight() { return ElevRight; }

    // Goal/setpoint setters
    
    public void setSetpoint(TrapezoidProfile.State nextSetpoint) { elevSetpointState = nextSetpoint; }
    
    public TrapezoidProfile.State getSetpoint() { return elevSetpointState; }
    
    public void setGoal(double goalState) { 
        elevGoalState = new TrapezoidProfile.State(goalState, 0); 
        setControl();
    }
    
    public TrapezoidProfile.State getGoal() { return elevGoalState; }
    
    public double getGoalValue() { return elevGoalState.position; }

    // Stop motors
    public void stopElev() {
        ElevLeft.stopMotor();
        ElevRight.stopMotor();
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return elevConstraints;
    }

    // Limit switches
    public boolean getLimitSwitchTop() { return topLimitSwitch.get(); }
    public boolean getLimitSwitchBot() { return botLimitSwitch.get(); }
    

    public double getElevatorHeight() {
        return ElevLeft.getPosition().getValueAsDouble();
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setControl() {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        TrapezoidProfile.State targetState = elevTrapezoidProfile.calculate(0.02, elevSetpointState, elevGoalState);
        m_request.Position = targetState.position;
        m_request.Velocity = targetState.velocity;
        setSetpoint(targetState);
        ElevLeft.setControl(m_request);
        ElevRight.setControl(m_request);
        elevatorLift.setLength(m_request.Position);
        elevatorLift.setAngle(90);
    }
}