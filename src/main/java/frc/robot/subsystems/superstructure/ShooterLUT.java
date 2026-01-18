package frc.robot.subsystems.superstructure;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterLUT {

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap angleMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap velocityMap = new InterpolatingDoubleTreeMap();

    public ShooterLUT() {
        //distance (meters) to RPM
        rpmMap.put(2.0, 3200.0);
        rpmMap.put(2.5, 3400.0);
        rpmMap.put(3.0, 3700.0);
        rpmMap.put(3.5, 4100.0);

        // distance (meters) to angle (degrees)
        angleMap.put(2.0, 38.0);
        angleMap.put(2.5, 36.0);
        angleMap.put(3.0, 33.0);
        angleMap.put(3.5, 30.0);

        // RPM to velocity (m/s)
        velocityMap.put(3200.0, 15.0);
        velocityMap.put(3400.0, 16.5);
        velocityMap.put(3700.0, 18.0);
        velocityMap.put(4100.0, 20.5);

    }

    public double getRPM(double distanceMeters) {
        return rpmMap.get(distanceMeters);
    }

    public double getAngle(double distanceMeters) {
        return angleMap.get(distanceMeters);
    }

    public double getVelocity(double distanceMeters) {
        return velocityMap.get(distanceMeters);
    }
}

