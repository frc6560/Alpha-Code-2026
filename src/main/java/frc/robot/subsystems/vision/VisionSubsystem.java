package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: add disabling cameras individually
// add disabling vision
public class VisionSubsystem extends SubsystemBase{
    private final List<LimelightVision> visionList;

    /** Wrapper class for all limelights */
    public VisionSubsystem(List<LimelightVision> visionList){
        this.visionList = visionList;
    }

    @Override
    public void periodic(){
        for(LimelightVision vision : visionList){
            vision.update();
        }
    }

    public Pose2d getLimelightPose(String cameraName) {
    for (LimelightVision vision : visionList) {
        if (vision.getName().equals(cameraName)) {
            return vision.getRobotPose2d();
        }
    }
    return new Pose2d(); // fallback
}

    public void hardReset(String cameraName){
        for(LimelightVision vision : visionList){
            if(vision.getName().equals(cameraName)){
                vision.hardUpdate();
            }
        }
    }
}
