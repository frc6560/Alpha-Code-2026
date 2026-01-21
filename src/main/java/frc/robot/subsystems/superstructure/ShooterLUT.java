package frc.robot.subsystems.superstructure;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLUT {

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap RPMtoVelocityMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap VelocitytoRPMMap = new InterpolatingDoubleTreeMap();

    public ShooterLUT() {

        //distance (meters) to RPM
        rpmMap.put(5.715, 3000.0);
        rpmMap.put(4.953, 2500.0);
        rpmMap.put(4.572, 2000.0);
        rpmMap.put(4.191, 1500.0);
        rpmMap.put(3.048, 1100.0);

        // distance (meters) to angle (degrees)
        angleMap.put(0.0, 45.0);
        angleMap.put(50.0, 45.0);
        

        // RPM to velocity (m/s)
        RPMtoVelocityMap.put(3200.0, 15.0);
        RPMtoVelocityMap.put(3400.0, 16.5);
        RPMtoVelocityMap.put(3700.0, 18.0);
        RPMtoVelocityMap.put(4100.0, 20.5);

        // Velocity to RPM (m/s)
        VelocitytoRPMMap.put(15.0, 3200.0);
        VelocitytoRPMMap.put(16.5, 3400.0);
        VelocitytoRPMMap.put(18.0, 3700.0);
        VelocitytoRPMMap.put(20.5, 4100.0);

    }


    public double getRPM(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    public double getAngle(double distanceMeters) {
        return angleMap.get(distanceMeters);
    }

    public double getVelocity(double distanceMeters) {
        return RPMtoVelocityMap.get(distanceMeters);
    }

    public double getRPMForVelocity(double velocity) {
        return VelocitytoRPMMap.get(velocity);
    }
}

