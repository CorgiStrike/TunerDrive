package frc.robot.Vision;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import static frc.robot.Constants.Vision.*;

public class Vision {
    private final AprilTagCamera[] pvCams;
    private Field2d feild = new Field2d();
    private Pose2d prevPose = new Pose2d();

    public Vision(List<PVCamera> camSettings ) {
        pvCams = 
            camSettings.stream()
                .map(
                    (settings) -> {
                        return new AprilTagCamera(settings);
                    })
                .toArray(AprilTagCamera[]::new);
        
    }

    public Pose2d getRobotPose() {
        Optional<EstimatedRobotPose> pose = pvCams[0].getEstimatedGlobalPose(prevPose);
        Pose3d robotPose = pose.get().estimatedPose;
        prevPose = new Pose2d(robotPose.getX(), robotPose.getY(), new Rotation2d(robotPose.getRotation().getZ()));
        return prevPose;
    }

    private PhotonPipelineResult preProcess(PhotonCamera cam, double ambiguityThreshold, double distanceFromLastEstimateScalar){
        PhotonPipelineResult pipelineData = cam.getLatestResult();
        HashMap<Integer, Double[]> ambiguityAverages = new HashMap<>();
        int avgLength = 100;

        if (!pipelineData.hasTargets()) return pipelineData;

        /* VERY IMPORTANT:
        * Clamp received vision timestamps to not pass the RIO time
        * If vision timestamps desync from the RIO, the pose estimate will have huge spasms
        * Issue for almost all of 2024 season - fixed at Champs 2024
        */
        pipelineData.setTimestampSeconds(
            Math.min(Timer.getFPGATimestamp(), pipelineData.getTimestampSeconds()));

        /*
        * Log the ambiguity of each tag the camera can see
        */
        int idx = 0;

        for (var tag : pipelineData.getTargets()) {
        if (!ambiguityAverages.containsKey(tag.getFiducialId())) {
            Double[] arr = new Double[avgLength];
            Arrays.fill(arr, -1.0);
            arr[0] = tag.getPoseAmbiguity();

            ambiguityAverages.put(tag.getFiducialId(), arr);
        } else {
            var arr = ambiguityAverages.get(tag.getFiducialId());
            System.arraycopy(arr, 0, arr, 1, arr.length - 1);
            arr[0] = tag.getPoseAmbiguity();
        }

        double avg = 0;
        double count = 0;
        for (Double a : ambiguityAverages.get(tag.getFiducialId())) {
            if (a >= 0) {
            avg += a;
            count++;
            }
        }

        avg /= count;

        PhotonTrackedTarget target =
            new PhotonTrackedTarget(
                tag.getYaw(),
                tag.getPitch(),
                tag.getArea(),
                tag.getSkew(),
                tag.getFiducialId(),
                tag.getBestCameraToTarget(),
                tag.getAlternateCameraToTarget(),
                avg,
                tag.getMinAreaRectCorners(),
                tag.getDetectedCorners());

        pipelineData.targets.set(idx, target);

        // Logging the ambiguity for each target can help with debugging potentially problematic
        // tag views
        Logger.recordOutput(
            "Vision/" + cam.getName() + "/target-" + target.getFiducialId() + "-avg-ambiguity",
            target.getPoseAmbiguity());

        idx++;
        }

        // Cut out targets with too high ambiguity
        pipelineData.targets.removeIf(target -> target.getPoseAmbiguity() > ambiguityThreshold);

        return pipelineData;
    }

    public Matrix<N3, N1> getXYThetaStdDev(EstimatedRobotPose pose, double trustCutOff) {
        double totalDistance = 0.0;
        int nonErrorTags = 0;
        double tagDistanceScalar = 1;
        double tagDistancePower = 0.33;

        // make the std dev greater based on how far away the tags are (trust estimates from further
        // tags less)
        // algorithm from frc6328 - Mechanical Advantage my beloved

        for (var tag : pose.targetsUsed) {
            var tagOnField = FIELD_LAYOUT.getTagPose(tag.getFiducialId());

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

    public VisionEstimate[] getEstimatedGlobalPose() {
        VisionEstimate[] estimates = new VisionEstimate[pvCams.length];
        int i = 0;
        for(AprilTagCamera cam:pvCams){
            PhotonPipelineResult raw = preProcess(cam.camera, cam.cam.ambiguityThreshhold, 2.0);
            var estimate = cam.photonPoseEstimator.update(raw);
            var stdDev = getXYThetaStdDev(estimate.get(), cam.cam.trustCutoff);
            var estimation = estimate.get();
            var pose = new Pose2d(estimation.estimatedPose.getX(), estimation.estimatedPose.getY(), new Rotation2d(estimation.estimatedPose.getRotation().getZ()));
            VisionEstimate finalEstimate = new VisionEstimate(prevPose, stdDev, estimation.timestampSeconds);
            estimates[i] = finalEstimate;
            i++;
        }
        return estimates;
    }

    // camera settings - define in Constants
    public record PVCamera(
        String name,
        Pose3d pose,
        Double ambiguityThreshhold,
        Double trustCutoff
    ) {}

    // pipeline-specific stuff
    public class AprilTagCamera {
        public PhotonPoseEstimator photonPoseEstimator;
        public PhotonCamera camera;
        public PVCamera cam;
        public AprilTagCamera(PVCamera cam) {
            this.cam = cam;
            PhotonCamera camera = new PhotonCamera(cam.name);
            this.camera = camera;
            this.photonPoseEstimator = new PhotonPoseEstimator(
                FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                new Transform3d(new Pose3d(), cam.pose));
        }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setLastPose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }
    }

    public record VisionEstimate(
        Pose2d estimatedPose,
        Matrix<N3, N1> stdDevs,
        Double timestamp
    ) {}
}