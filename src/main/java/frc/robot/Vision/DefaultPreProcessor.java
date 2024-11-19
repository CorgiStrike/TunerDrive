package frc.robot.Vision;

import java.util.HashMap;
import java.util.LinkedList;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;

public class DefaultPreProcessor implements Preprocessor {
    HashMap<Integer, LinkedList<Double>> rollingAverages = new HashMap<>();
    double ambiguityThreshold;
    double distanceScalar;
    int avgLength;

    public DefaultPreProcessor(double ambiguityThreshold, double distanceScalar, int avgLength) {
        this.ambiguityThreshold = ambiguityThreshold;
        this.distanceScalar = distanceScalar;
        this.avgLength = avgLength;
    }

    @Override
    public PhotonPipelineResult preprocess(PhotonPipelineResult in) {
        if (!in.hasTargets()) return in;

        /* VERY IMPORTANT:
        * Clamp received vision timestamps to not pass the RIO time
        * If vision timestamps desync from the RIO, the pose estimate will have huge spasms
        * Issue for almost all of 2024 season - fixed at Champs 2024
        */
        in.setTimestampSeconds(
            Math.min(Timer.getFPGATimestamp(), in.getTimestampSeconds()));

        in.targets.removeIf((target) -> addAndComputeAverage(target.getFiducialId(), target.getPoseAmbiguity()) > ambiguityThreshold);

        return in;
    }
    

    private double addAndComputeAverage(int id, double current) {
        if (!rollingAverages.containsKey(id)) {
            var list = new LinkedList<Double>();

            list.add(current);
            rollingAverages.put(id, list);

            return current;
        }

        var history = rollingAverages.get(id);

        history.addFirst(current);

        if (history.size() > avgLength) {
            history.removeLast();
        }

        return history.stream().reduce(0.0, (a, b) -> a + b).doubleValue() / history.size();
    }
}
