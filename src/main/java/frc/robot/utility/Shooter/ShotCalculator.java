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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurretConstants;


public class ShotCalculator {
    public double flywheelRPM;
    private double hoodAzimuth; // in radians
    private double turretAngle; // in radians

    private static final double TIME_PARAMETER = 0.03; // seconds into the future to project

    private static final InterpolatingDoubleTreeMap hoodAzimuthMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    public Translation2d virtualTargetPose; // for SOTM

    /** A util class for outputting shooter state values, even while the robot is moving!*/
    public ShotCalculator() {
        this.flywheelRPM = 0;
        this.hoodAzimuth = 0;
        this.turretAngle = 0;
        this.virtualTargetPose = new Translation2d();
    }

    public double getHoodAzimuth() {
        return hoodAzimuth;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public Translation2d getVirtualTargetPose() {
        return virtualTargetPose;
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
        
        // Calculates projected position
        Pose2d projectedPosition = currentRobotPose.exp(
            new Twist2d(
                fieldVelocity.vxMetersPerSecond * TIME_PARAMETER,
                fieldVelocity.vyMetersPerSecond * TIME_PARAMETER,
                fieldVelocity.omegaRadiansPerSecond * TIME_PARAMETER
            )
        );

        // for now, use the robot pose as the turret pose. change once you get constants.
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
        
        // Calculates a virtual target iteratively based upon our parameters.
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

        // Calculates hood angle and flywheel RPM from virtual target
        hoodAzimuth = hoodAzimuthMap.get(distanceToTarget);
        flywheelRPM = flywheelRPMMap.get(distanceToTarget);
        turretAngle = MathUtil.angleModulus(Math.atan2(
            virtualTargetPose.getY() - turretPose.getY(),
            virtualTargetPose.getX() - turretPose.getX()
        ) - projectedPosition.getRotation().getRadians());
    }
}
