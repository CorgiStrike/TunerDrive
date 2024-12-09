package frc.robot.Vision;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Vision.Vision.VisionEstimate;

public class DefaultPostProcessor implements Postprocessor {
    AprilTagFieldLayout fieldLayout;
    double trustCutOff;

    public DefaultPostProcessor(AprilTagFieldLayout fieldLayout, double trustCutOff) {
        this.fieldLayout = fieldLayout;
        this.trustCutOff = trustCutOff;
    }

    @Override
    public VisionEstimate postProcess(EstimatedRobotPose pose) {
        var stdDev = getXYThetaStdDev(pose, trustCutOff);

        return new VisionEstimate(
            pose.estimatedPose.toPose2d(), 
            stdDev, 
            pose.timestampSeconds
        );
    }

    private Matrix<N3, N1> getXYThetaStdDev(EstimatedRobotPose pose, double trustCutOff) {
        double totalDistance = 0.0;
        int nonErrorTags = 0;
        double tagDistanceScalar = 1;
        double tagDistancePower = 0.33;

        // make the std dev greater based on how far away the tags are (trust estimates from further
        // tags less)
        // algorithm from frc6328 - Mechanical Advantage my beloved

        for (var tag : pose.targetsUsed) {
            var tagOnField = fieldLayout.getTagPose(tag.getFiducialId());

            if (tagOnField.isPresent()) {
                totalDistance +=
                    pose.estimatedPose
                        .toPose2d()
                        .getTranslation()
                        .getDistance(tagOnField.get().toPose2d().getTranslation());
                nonErrorTags++;
            }
        }
        double avgDistance = totalDistance / nonErrorTags;

        avgDistance *= tagDistanceScalar;

        double xyStdDev = Math.pow(avgDistance, tagDistancePower) / nonErrorTags;
        double thetaStdDev = Math.pow(avgDistance, tagDistancePower) / nonErrorTags;

        if (avgDistance >= trustCutOff) {
            return VecBuilder.fill(10000, 10000, 10000);
        }

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
    
}
