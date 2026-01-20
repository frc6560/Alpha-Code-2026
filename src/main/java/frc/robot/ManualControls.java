package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class ManualControls {

    // private final XboxController secondXbox;
    private final XboxController secondXbox;
    private final XboxController firstXbox;

    
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }

      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.01);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }
    public ManualControls(XboxController firstXbox, XboxController secondXbox) {
        this.secondXbox = secondXbox;
        this.firstXbox = firstXbox;
        
    }

    // climb

    public boolean getClimbDown() {
      return secondXbox.getRightY() > 0.7; 
        // return secondXbox.getLeftStickButton(); 
    }

    public boolean getClimbUp() {
      return secondXbox.getRightY() < -0.7;
        // return secondXbox.getRightStickButton(); 
    }

    public boolean BallOut(){
      return secondXbox.getRightTriggerAxis() > 0.5;
    }


    // pipe and ball grabber 

    // shifted for ball
    public boolean runIntake(){
      return secondXbox.getLeftBumperButton();
    }

    public boolean zeroNoAprilTagsGyro() {
      return secondXbox.getStartButton();
    }
}
