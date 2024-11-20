package frc.robot.Vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public VisionEstimate[] getEstimatedGlobalPose() {
        VisionEstimate[] estimates = Arrays.stream(pvCams)
            .map((cam) -> cam.getEstimate(prevPose))
            .filter((est) -> est.isPresent())
            .map((est) -> est.get())
            .toArray(VisionEstimate[]::new);

        updateField2d(estimates);

        return estimates;
    }

    private void updateField2d(VisionEstimate[] estimates) {
        var obj = feild.getObject("Cam Estimations");
        obj.setPoses(Arrays.stream(estimates).map((e) -> e.estimatedPose()).toList());
    }

    // camera settings - define in Constants
    public record PVCamera(
        String name,
        Pose3d pose,
        Preprocessor pre,
        Postprocessor post
    ) {}

    // pipeline-specific stuff
    public class AprilTagCamera {
        PhotonPoseEstimator photonPoseEstimator;
        PhotonCamera camera;
        PVCamera cam;

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

        public Optional<VisionEstimate> getEstimate(Pose2d lastPose) {
            photonPoseEstimator.setLastPose(lastPose);

            var est = photonPoseEstimator.update(cam.pre().preprocess(camera.getLatestResult()));

            if (est.isPresent()) {
                return Optional.of(cam.post().postProcess(est.get()));
            }

            return Optional.empty();
        }
    }

    public record VisionEstimate(
        Pose2d estimatedPose,
        Matrix<N3, N1> stdDevs,
        Double timestamp
    ) {}
}