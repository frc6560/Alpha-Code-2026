package frc.robot.utility.Shooter;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;

public class ShotCalculator {

    /** State container for turret position and velocity. */
    public record TurretState(double positionRadians, double velocityRadiansPerSecond) {}

    /** State container for hood position and velocity. */
    public record HoodState(double positionRadians, double velocityRadiansPerSecond) {}

    /** Combined state for the entire shooter system. */
    public record ShooterState(
        TurretState turret,
        HoodState hood,
        double flywheelRPM,
        Translation2d virtualTargetPose
    ) {}

    public double flywheelRPM;
    private double hoodAzimuth; // in radians
    private double turretAngle; // in radians
    private double hoodVelocity; // rad/s
    private double turretVelocity; // rad/s

    private static final double TIME_PARAMETER = 0.03; // seconds into the future to project

    // low-pass filter coefficient (0-1, higher = more smoothing)
    private static final double VELOCITY_FILTER_ALPHA = 0.8;

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public Translation2d virtualTargetPose; // for SOTM

    // state for numerical differentiation
    private double prevHoodAzimuth = 0;
    private double prevTurretAngle = 0;
    private double prevTimestamp = 0;
    private double filteredHoodVelocity = 0;
    private double filteredTurretVelocity = 0;
    private boolean hasInitialized = false;

    /** A util class for outputting shooter state values, even while the robot is moving! */
    public ShotCalculator() {
        this.flywheelRPM = 0;
        this.hoodAzimuth = 0;
        this.turretAngle = 0;
        this.hoodVelocity = 0;
        this.turretVelocity = 0;
        this.virtualTargetPose = new Translation2d();
    }

    public double getHoodAzimuth() {
        return hoodAzimuth;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getHoodVelocity() {
        return hoodVelocity;
    }

    public double getTurretVelocity() {
        return turretVelocity;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public Translation2d getVirtualTargetPose() {
        return virtualTargetPose;
    }

    /** Returns the complete shooter state including positions and velocities. */
    public ShooterState getState() {
        return new ShooterState(
            new TurretState(turretAngle, turretVelocity),
            new HoodState(hoodAzimuth, hoodVelocity),
            flywheelRPM,
            virtualTargetPose
        );
    }

    /** Calculates the shot parameters based on current robot pose and field velocity. */
    public void calculate(Pose2d currentRobotPose, 
                            ChassisSpeeds fieldVelocity) {
        // Gets our target pose
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            return;
        }
        Translation2d targetPose = (alliance.get() == Alliance.Blue) ? 
                                            FieldConstants.BLUE_HUB_CENTER : 
                                            FieldConstants.RED_HUB_CENTER;
        
        ChassisSpeeds robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldVelocity.vxMetersPerSecond,
            fieldVelocity.vyMetersPerSecond,
            fieldVelocity.omegaRadiansPerSecond,
            currentRobotPose.getRotation()
        );
        
        // calculates projected position due to sensor lag
        Pose2d projectedPosition = currentRobotPose.exp(
            new Twist2d(
                robotVelocity.vxMetersPerSecond * TIME_PARAMETER,
                robotVelocity.vyMetersPerSecond * TIME_PARAMETER,
                robotVelocity.omegaRadiansPerSecond * TIME_PARAMETER
            )
        );

        // gets the turret's robot relative transform
        Transform2d turretTransform = new Transform2d(
            TurretConstants.ROBOT_RELATIVE_TURRET.getX(), 
            TurretConstants.ROBOT_RELATIVE_TURRET.getY(),
            new Rotation2d()
        );


        // gets the turret's field relative velocity
        Pose2d turretPose = projectedPosition.transformBy(turretTransform);
        double angleOffset = Math.atan2(turretTransform.getY(), turretTransform.getX());
        double r = Math.hypot(turretTransform.getX(), turretTransform.getY());

        double turretVx = fieldVelocity.vxMetersPerSecond
                        + (-r * fieldVelocity.omegaRadiansPerSecond * Math.sin(projectedPosition.getRotation().getRadians() + angleOffset));
        double turretVy = fieldVelocity.vyMetersPerSecond
                        + (r * fieldVelocity.omegaRadiansPerSecond * Math.cos(projectedPosition.getRotation().getRadians() + angleOffset));
        
        // calculates a virtual target iteratively based upon our parameters.
        virtualTargetPose = targetPose;
        double timeOfFlight = 0;
        double distanceToTarget = turretPose.getTranslation().getDistance(targetPose);

        for(int i = 0; i < 20; i++){
            timeOfFlight = timeOfFlightMap.get(distanceToTarget);
            virtualTargetPose = targetPose.minus(
                new Translation2d(
                    turretVx * timeOfFlight,
                    turretVy * timeOfFlight
                )
            );
            distanceToTarget = turretPose.getTranslation().getDistance(virtualTargetPose);
        }

        // calculates hood angle and flywheel RPM from virtual target
        double newHoodAzimuth = hoodAzimuthMap.get(distanceToTarget);
        flywheelRPM = flywheelRPMMap.get(distanceToTarget);
        double newTurretAngle = MathUtil.angleModulus(Math.atan2(
            virtualTargetPose.getY() - turretPose.getY(),
            virtualTargetPose.getX() - turretPose.getX()
        ) - projectedPosition.getRotation().getRadians());

        // Calculate velocities using numerical differentiation with low-pass filtering
        double currentTime = Timer.getFPGATimestamp();
        if (hasInitialized) {
            double dt = currentTime - prevTimestamp;
            if (dt > 1e-6) { // Avoid division by zero
                // Raw velocity from backward difference
                double rawHoodVelocity = MathUtil.angleModulus(newHoodAzimuth - prevHoodAzimuth) / dt;
                double rawTurretVelocity = MathUtil.angleModulus(newTurretAngle - prevTurretAngle) / dt;

                // Exponential moving average filter: filtered = alpha * prev + (1 - alpha) * raw
                filteredHoodVelocity = VELOCITY_FILTER_ALPHA * filteredHoodVelocity
                                     + (1 - VELOCITY_FILTER_ALPHA) * rawHoodVelocity;
                filteredTurretVelocity = VELOCITY_FILTER_ALPHA * filteredTurretVelocity
                                       + (1 - VELOCITY_FILTER_ALPHA) * rawTurretVelocity;
            }
        } else {
            hasInitialized = true;
        }

        prevHoodAzimuth = newHoodAzimuth;
        prevTurretAngle = newTurretAngle;
        prevTimestamp = currentTime;

        hoodAzimuth = newHoodAzimuth;
        turretAngle = newTurretAngle;
        hoodVelocity = filteredHoodVelocity;
        turretVelocity = filteredTurretVelocity;
    }

    /** Resets the velocity filter state. Call this when re-enabling or after long pauses. */
    public void resetFilter() {
        hasInitialized = false;
        filteredHoodVelocity = 0;
        filteredTurretVelocity = 0;
    }
}
