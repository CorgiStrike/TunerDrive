package frc.robot.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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

    // camera settings - define in Constants
    public record PVCamera(
        String name,
        Pose3d pose,
        Double ambiguityThreshhold
    ) {}

    // pipeline-specific stuff
    public class AprilTagCamera {
        private PhotonPoseEstimator photonPoseEstimator;
        public AprilTagCamera(PVCamera cam) {
            PhotonCamera camera = new PhotonCamera(cam.name);
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
}