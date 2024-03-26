package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.PhotonVisionTags;

public class PhotonVisionSubsystem {
   
    PhotonVisionTags limelight;
   
    public void periodic() {
    var result = limelight.getLatestPipeline();
        //if (result.getMultiTagResult().estimatedPose.isPresent) {
        //Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
        //}

    }
    public static void updateSmartDashboard() {
        PhotonPipelineResult pipelineResult = PhotonVisionTags.getLatestPipeline();
        if (PhotonVisionTags.hasTarget(pipelineResult)) {
            int targetCount = 0;
            for (PhotonTrackedTarget target : PhotonVisionTags.getTargets(pipelineResult)) {
                SmartDashboard.putNumber("AprilTag ID " + targetCount, PhotonVisionTags.getTargetId(target));
                targetCount++;
            }
            SmartDashboard.putNumber("Total AprilTags Detected", targetCount);
        } else {
            SmartDashboard.putNumber("Total AprilTags Detected", 0);
        }
    }
}