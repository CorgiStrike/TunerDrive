package frc.robot.Vision;

import org.photonvision.targeting.PhotonPipelineResult;

public interface Preprocessor {
    PhotonPipelineResult preprocess();
}
