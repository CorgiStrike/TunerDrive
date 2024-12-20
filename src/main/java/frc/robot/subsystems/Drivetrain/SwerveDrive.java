package frc.robot.subsystems.Drivetrain;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
    private static final double simLoopPeriod = 0.005; // 5 ms
    private List<PVCamera> camSettings;
    private Vision vision;
    private Notifier simNotifier = null;
    private double lastSimTime;
    private Field2d field = new Field2d();
    private SwerveDrivePoseEstimator odometry;

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
        odometry = new SwerveDrivePoseEstimator(m_kinematics, m_fieldRelativeOffset, m_modulePositions, new Pose2d());
    }
    public SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initVision();
        odometry = new SwerveDrivePoseEstimator(m_kinematics, m_fieldRelativeOffset, m_modulePositions, new Pose2d());
    }

    private void initVision() {
        camSettings = List.of(
            Constants.Vision.LEFT_SHOOTER_CAM,
            Constants.Vision.RIGHT_SHOOTER_CAM,
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
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(simLoopPeriod);
    }

    private void updateVisionPose() {
        var estimates = vision.getEstimatedGlobalPose();
        for (Vision.VisionEstimate estimate:estimates){
            if(!(estimate==null)) odometry.addVisionMeasurement(estimate.estimatedPose(),estimate.timestamp(), estimate.stdDevs());
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
        odometry.update(getPigeon2().getRotation2d(),m_modulePositions);
        field.setRobotPose(odometry.getEstimatedPosition());
    }
}
