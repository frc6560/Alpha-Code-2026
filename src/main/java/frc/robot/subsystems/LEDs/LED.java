package frc.robot.subsystems.LEDs;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static final int CANDLE_ID = 41;
    private static final int CANDLE_LEDS = 8;
    private static final int STRIP_LEDS = 1; // Update this with your actual strip LED count
    private static final int TOTAL_LEDS = CANDLE_LEDS + STRIP_LEDS;
    
    private final CANdle candle;
    
    // Colors
    private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);
    
    // Control requests
    private final SolidColor greenControl;
    private final SolidColor offControl;
    
    public LED() {
        candle = new CANdle(CANDLE_ID);
        
        // SolidColor(startIndex, ledCount)
        greenControl = new SolidColor(0, TOTAL_LEDS).withColor(GREEN);
        offControl = new SolidColor(0, TOTAL_LEDS).withColor(OFF);
        
        off();
    }
    
    public void setGreen() {
        candle.setControl(greenControl);
    }
    
    public void off() {
        candle.setControl(offControl);
    }
}