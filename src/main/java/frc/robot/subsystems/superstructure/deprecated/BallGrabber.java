
package frc.robot.subsystems.superstructure.deprecated;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix6.hardware.CANrange; 

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
// import com.robot.Constants; 

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;
import static edu.wpi.first.units.Units.Degree;

public class BallGrabber extends SubsystemBase {
    private final TalonFX grabberMotor; 
    private final CANrange grabberMotorRange; // Range for TalonFX IDs

 
 
    private TalonFXConfiguration fxConfig;
    


    private static final double INTAKE_SPEED = -0.5;
    private static final double OUTTAKE_SPEED = 0.7; 

    private static final double MAX_CURRENT_RUNNING = 30; 
    private static final double BALL_DETECTION_DISTANCE = 0.4; //  threshold to detect if a ball is present
    
    private long lastBurstTime = 0;
    private static final long BURST_DURATION = 500; // milliseconds
    private static final long BURST_INTERVAL = 1500; // milliseconds between bursts
    private boolean isBursting = false;
    
    private Timer stopDelayTimer = new Timer();
    private boolean waitingToStop = false;


    //   private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("BallGrabber");
    //   private final NetworkTableEntry ntCurrent = ntTable.getEntry("Ball Grabber Current");
    //   private final NetworkTableEntry ntVoltage = ntTable.getEntry("Ball Grabber Voltage");
    //   private final NetworkTableEntry ntSpeed = ntTable.getEntry("Ball Grabber Speed");
    //   private final NetworkTableEntry ntDutyCycle = ntTable.getEntry("Ball Grabber Velocity");

    public BallGrabber() {
        this.grabberMotor = new TalonFX(46, "Canivore"); 
        this.grabberMotorRange = new CANrange(47, "Canivore"); 
        
    }


    public void periodic(){
        if (waitingToStop && stopDelayTimer.hasElapsed(0.5)) {
            stop(); // Actually stop the grabber
            stopDelayTimer.stop();
            waitingToStop = false;
        }
        
    }

    public void runIntake(){
        grabberMotor.set(-(INTAKE_SPEED));
    }

    public void runOuttake(){
        grabberMotor.set(-(OUTTAKE_SPEED));
    }


public void stop(){
    long currentTime = System.currentTimeMillis();
    
    if (!isBursting && (currentTime - lastBurstTime) >= BURST_INTERVAL) {
        // Start burst
        grabberMotor.set(0.15);
        isBursting = true;
        lastBurstTime = currentTime;
    } else if (isBursting && (currentTime - lastBurstTime) >= BURST_DURATION) {
        // End burst
        grabberMotor.set(0.03);
        isBursting = false;
    }
}

    public void startDelayedStop() {
        stopDelayTimer.reset();
        stopDelayTimer.start();
        waitingToStop = true;
    }

    public double getMotorVelocity(){
        return grabberMotor.get();
    }

    public boolean hasBall() {
        var range = grabberMotorRange.getDistance().getValueAsDouble();
        return range < BALL_DETECTION_DISTANCE;
    }
    public double getDutyCycle() {
        return grabberMotor.get(); 
    }

}
