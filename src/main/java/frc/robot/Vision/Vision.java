package frc.robot.Vision;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;

// UNFINISHED - COMMENTING AND FORMATING AFTER FINISHING
public class Vision {
    private final AprilTagCamera[] pvCams;
    private Field2d feild = new Field2d();

    public Vision(List<PVCamera> camSettings ) {
        pvCams = 
            camSettings.stream()
                .map(
                    (settings) -> {
                        return new AprilTagCamera(settings);
                    })
                .toArray(AprilTagCamera[]::new);
        
    }

    public Field2d getFeild() {
        return feild;
    }

    public EstimatedRobotPose getRobotPose() {
        Double ambiguity = 1.0;
        EstimatedRobotPose pose = null;
        for(var cam : pvCams){
            EstimatedRobotPose tempPose = cam.getCameraPose();
            if( cam.getPoseAmbiguity() < ambiguity && tempPose != null){
                pose = tempPose;
                ambiguity = cam.getPoseAmbiguity();
            }
        }
        return pose;
    }
    
    // camera settings - define in Constants
    public record PVCamera(
        String name,
        Pose3d pose,
        Double ambiguityThreshhold
    ) {}

    // pipeline-specific stuff
    public class AprilTagCamera {
        private PhotonCamera camera;                // pv pipeline provider
        private Pose3d camPose;           // poses to return, last pose for fallback
        private PhotonPoseEstimator poseEstimator;  // pose estimator
        private Double ambiguityThreshhold;         // 0-1, how ambiguous a tag can be before it is ignored
        private EstimatedRobotPose lastEstimation;  // last estimated pose from photonvision

        // constructor
        public AprilTagCamera(PVCamera cam) {
            // set class vars
            this.camera = new PhotonCamera(cam.name);
            this.camera.setDriverMode(false);
            this.camPose = cam.pose;
            this.ambiguityThreshhold = cam.ambiguityThreshhold;
            this.lastEstimation = new EstimatedRobotPose(null, 0, null, null); // Default null pose
            // create pose estimator
            this.poseEstimator = new PhotonPoseEstimator(
                        Constants.Vision.FIELD_LAYOUT,
                        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,    // change layout in Constants
                        camera,
                        new Transform3d(camPose.getTranslation(),
                        camPose.getRotation()));
            poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        }

        // get robot pose from ONE camera
        public EstimatedRobotPose getCameraPose() {
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()) {  // make sure tags exist, otherwise error is thrown and both robot AND photonvision crashes
                for(var target : result.getTargets()){
                    Double ambiguity = target.getPoseAmbiguity();
                    if(ambiguity > this.ambiguityThreshhold || ambiguity == -1){
                        result.targets.remove(target);  // remove target if it is too ambiguous
                    }
                }
                if(!result.hasTargets()){
                    return lastEstimation;    // return last pose if all targets were too ambiguous
                }
                PhotonTrackedTarget tag = result.getBestTarget();
                // make sure correct field/tags are being used
                if (Constants.Vision.FIELD_LAYOUT.getTagPose(tag.getFiducialId()).isPresent()) {
                    lastEstimation = poseEstimator.update().get();
                    
                }
            }
            return lastEstimation;
        }

        // get last pose ambiguity
        public Double getPoseAmbiguity() {
            Double avgAmbiguity = 0.0;
            Integer numTargets = 0;
            if(lastEstimation==null) return 1.0;
            for(var tag : lastEstimation.targetsUsed) {
                avgAmbiguity += tag.getPoseAmbiguity();
                numTargets++;
            }
            avgAmbiguity /= numTargets;
            return avgAmbiguity;
        }
    }
}