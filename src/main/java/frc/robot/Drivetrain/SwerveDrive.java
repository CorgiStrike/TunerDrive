package frc.robot.Drivetrain;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.Vision.Vision;
import frc.robot.Vision.Vision.PVCamera;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class SwerveDrive extends SwerveDrivetrain{
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private List<PVCamera> camSettings;
    private Vision vision;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d field = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initVision();
    }
    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initVision();
    }

    private void initVision() {
        camSettings = List.of(
            //Constants.Vision.LEFT_SHOOTER_CAM,
            //Constants.Vision.RIGHT_SHOOTER_CAM,
            Constants.Vision.LEFT_INTAKE_CAM,
            Constants.Vision.RIGHT_INTAKE_CAM
            );
        vision = new Vision(camSettings);
        SmartDashboard.putData("Vision Pose", field);
    }

    public Runnable getRequestRunnable(Supplier<SwerveRequest> requestSupplier) {
        return () -> this.setControl(requestSupplier.get());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void updateVisionPose() {
        var pose = vision.getRobotPose();
        if (pose != null) {
            Pose3d rawPose = pose.estimatedPose;
            Pose2d calcPose = new Pose2d(rawPose.getX(),rawPose.getY(),new Rotation2d(rawPose.getRotation().getZ()));
            field.setRobotPose(calcPose);
            this.addVisionMeasurement(calcPose, pose.timestampSeconds);
        
        }
    }

    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        updateVisionPose();
    }
}
