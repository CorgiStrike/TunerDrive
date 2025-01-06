package frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.SMF.StateMachine;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends StateMachine<CommandSwerveDrivetrain.State>{
    public final SwerveDrive swerveDrive;
    private final PIDController anglePID = new PIDController(0.1, 0.0, 0.2);
    private double maxSpeed = 0.0, maxAngularRate = 0.0;
    private Supplier<Double> xSupplier = null, ySupplier = null, turnSupplier = null;
    private final LegacySwerveModuleConstants[] modules;
    private final BooleanSupplier mirrorPath;

    public CommandSwerveDrivetrain(LegacySwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, double maxSpeed, double maxAngularRate, BooleanSupplier mirrorPath, LegacySwerveModuleConstants... modules) {
        super("CommandSwerveDrive", State.UNDETERMINED, State.class);
        swerveDrive = new SwerveDrive(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.modules = modules;
        this.mirrorPath = mirrorPath;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        registerStateTransitions();
        registerStateCommands();
        configurePathPlanner();
    }
    public CommandSwerveDrivetrain(LegacySwerveDrivetrainConstants driveTrainConstants, double maxSpeed, double maxAngularRate, BooleanSupplier mirrorPath, LegacySwerveModuleConstants... modules) {
        super("RobotContainer", State.UNDETERMINED, State.class);
        swerveDrive = new SwerveDrive(driveTrainConstants, modules);
        this.modules = modules;
        this.mirrorPath = mirrorPath;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        registerStateTransitions();
        registerStateCommands();
        configurePathPlanner();
    }

    public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
        return run(swerveDrive.getRequestRunnable(requestSupplier));
    }

    private void drive(Double maxSpeed, Double maxAngularRate, Double xValue, Double yValue, Double turnValue, Boolean fieldOriented) {
        if (fieldOriented) {
            final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
            applyRequest(()->drive.withVelocityX(xValue * maxSpeed).withVelocityY(yValue * maxSpeed).withRotationalRate(turnValue * maxAngularRate)).schedule();
        }

        else {
            final LegacySwerveRequest.RobotCentric drive = new LegacySwerveRequest.RobotCentric()
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
            applyRequest(()->drive.withVelocityX(xValue * maxSpeed).withVelocityY(yValue * maxSpeed).withRotationalRate(turnValue * maxAngularRate)).schedule();
        }
        
    }

    private void driveChassisSpeeds(ChassisSpeeds speeds) {
        drive(maxSpeed, maxAngularRate, speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
    }

    private ChassisSpeeds getChassisSpeeds() {
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[] {
            new Translation2d(modules[0].LocationX, modules[0].LocationY),
            new Translation2d(modules[1].LocationX, modules[1].LocationY),
            new Translation2d(modules[2].LocationX, modules[2].LocationY),
            new Translation2d(modules[3].LocationX, modules[3].LocationY)
        });
        return kinematics.toChassisSpeeds(swerveDrive.getState().ModuleStates);
    }

    public void configureBindings(Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> turnSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.turnSupplier = turnSupplier;
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    @Override
    protected void update() {
        swerveDrive.periodic();
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.TRAVERSING);
    }


    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(() -> {
            drive(0.0, 0.0, 0.0, 0.0, 0.0, false);
        }));

        registerStateCommand(State.TRAVERSING, new RunCommand(() -> {
            drive(maxSpeed, maxAngularRate, xSupplier.get(), ySupplier.get(), turnSupplier.get(), true);
        }));
    }

    private void configurePathPlanner() {
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            swerveDrive::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            AutoConstants.config, // The robot configuration
            mirrorPath::getAsBoolean, // Whether to mirror the path
            this // Reference to this subsystem to set requirements
    );
    }

    private void pointTowardsAngle(Double angle) {
        anglePID.setSetpoint(angle);
        Double turnValue = anglePID.calculate(swerveDrive.getPigeon2().getYaw().getValueAsDouble());
        drive(maxSpeed, maxAngularRate, xSupplier.get(), ySupplier.get(), turnValue, true);
    }

    public Pose2d getPose() {
        return swerveDrive.getState().Pose;
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        TRAVERSING
    }
}
