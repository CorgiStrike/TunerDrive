package frc.robot.Vision;

import org.photonvision.EstimatedRobotPose;

public interface Postprocessor {
    Vision.VisionEstimate postProcess(EstimatedRobotPose pose);
}
