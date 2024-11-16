package frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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
import frc.robot.Vision.Limelight;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends StateMachine<CommandSwerveDrivetrain.State>{
    public final SwerveDrive swerveDrive;
    private final Limelight limelight = new Limelight("limelight"); 
    private final PIDController anglePID = new PIDController(0.1, 0.0, 0.0);
    private double maxSpeed = 0.0, maxAngularRate = 0.0;
    private Supplier<Double> xSupplier = null, ySupplier = null, turnSupplier = null;
    private final SwerveModuleConstants[] modules;
    private final BooleanSupplier mirrorPath;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, double maxSpeed, double maxAngularRate, BooleanSupplier mirrorPath, SwerveModuleConstants... modules) {
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
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double maxSpeed, double maxAngularRate, BooleanSupplier mirrorPath, SwerveModuleConstants... modules) {
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

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(swerveDrive.getRequestRunnable(requestSupplier));
    }

    private void drive(Double maxSpeed, Double maxAngularRate, Double xValue, Double yValue, Double turnValue, Boolean fieldOriented) {
        if (fieldOriented) {
            final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
            applyRequest(()->drive.withVelocityX(xValue * maxSpeed).withVelocityY(yValue * maxSpeed).withRotationalRate(turnValue * maxAngularRate)).schedule();
        }

        else {
            final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
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
        addOmniTransition(State.AUTO_INTAKE);
        addOmniTransition(State.IDLE);
        addOmniTransition(State.TRAVERSING);
        addOmniTransition(State.AMP);
        addOmniTransition(State.SOURCE);
        addOmniTransition(State.SPEAKER_AA);
    }


    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(() -> {
            drive(0.0, 0.0, 0.0, 0.0, 0.0, false);
        }));

        registerStateCommand(State.TRAVERSING, new RunCommand(() -> {
            drive(maxSpeed, maxAngularRate, xSupplier.get(), ySupplier.get(), turnSupplier.get(), true);
        }));

        registerStateCommand(State.AUTO_INTAKE, new RunCommand(() -> {
            autoIntakeDrive();
        }).repeatedly());

        registerStateCommand(State.AMP, new RunCommand(() -> {
            pointTowardsAngle(0.0);
        }).repeatedly());

        registerStateCommand(State.SOURCE, new RunCommand(() -> {
            pointTowardsAngle(0.0);
        }).repeatedly());

        registerStateCommand(State.SPEAKER_AA, new RunCommand(() -> {
            pointTowardsAngle(0.0);
        }).repeatedly());
    }

    private void configurePathPlanner() {
        final Double driveBaseRadius = Math.hypot(modules[0].LocationX, modules[0].LocationY);
        
        AutoBuilder.configureHolonomic(
            this::getPose,
            swerveDrive::seedFieldRelative,
            this::getChassisSpeeds,
            this::driveChassisSpeeds,
            new HolonomicPathFollowerConfig(
                AutoConstants.TRANSLATION_PID, 
                AutoConstants.ANGLE_PID, 
                maxSpeed, 
                driveBaseRadius, 
                new ReplanningConfig()),
            mirrorPath,
            this
        );
    }

    private void autoIntakeDrive() {
        var tx = limelight.getTX();
        var ta = limelight.getTA();

        Double drive = 0.0;
        Double turn = 0.0;

        if (Math.abs(tx)<8||ta<5) {
            drive = -0.4;
        }
        if(tx > 20) turn = -tx/100;
        else turn = -tx/80;

        if(ta == 0){
            drive = 0.0;
            turn = 0.0;
        }

        drive(maxSpeed, maxAngularRate, drive, 0.0, turn, false);
    }

    private void pointTowardsAngle(Double angle) {
        anglePID.setSetpoint(angle);
        Double turnValue = anglePID.calculate(swerveDrive.getPigeon2().getAngle());
        drive(maxSpeed, maxAngularRate, xSupplier.get(), ySupplier.get(), turnValue, true);
    }

    public Pose2d getPose() {
        return swerveDrive.getState().Pose;
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        TRAVERSING,
        AUTO_INTAKE,
        AMP,
        SOURCE,
        SPEAKER_AA
    }
}
