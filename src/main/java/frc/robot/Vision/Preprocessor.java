package frc.robot.Vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface Preprocessor {
    PhotonPipelineResult preprocess(PhotonPipelineResult in);
}
