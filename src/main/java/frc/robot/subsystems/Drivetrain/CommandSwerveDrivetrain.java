package frc.robot.subsystems.Drivetrain;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Drivetrain.SwerveDrive;
import frc.robot.SMF.StateMachine;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends StateMachine<CommandSwerveDrivetrain.State>{
    public final SwerveDrive swerveDrive;
    private double maxSpeed = 0.0, maxAngularRate = 0.0;
    private Supplier<Double> xSupplier = null, ySupplier = null, turnSupplier = null;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, double maxSpeed, double maxAngularRate, SwerveModuleConstants... modules) {
        super("CommandSwerveDrive", State.UNDETERMINED, State.class);
        swerveDrive = new SwerveDrive(driveTrainConstants, OdometryUpdateFrequency, modules);
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        registerStateTransitions();
        registerStateCommands();
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double maxSpeed, double maxAngularRate, SwerveModuleConstants... modules) {
        super("RobotContainer", State.UNDETERMINED, State.class);
        swerveDrive = new SwerveDrive(driveTrainConstants, modules);
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;
        registerStateTransitions();
        registerStateCommands();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(swerveDrive.getRequestRunnable(requestSupplier));
    }

    private void drive(double maxSpeed, double maxAngularRate, Double xValue, Double yValue, Double turnValue) {
        final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric driving in open loop
        applyRequest(()->drive.withVelocityX(xValue).withVelocityY(yValue).withRotationalRate(turnValue));
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

    private void registerStateTransitions() {
        addTransition(State.IDLE, State.AUTO_INTAKE);

        addOmniTransition(State.IDLE);
        addOmniTransition(State.TRAVERSING);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new RunCommand(() -> swerveDrive.periodic()));
        registerStateCommand(State.TRAVERSING, new RunCommand(() -> {
            swerveDrive.periodic();
            drive(maxSpeed, maxAngularRate, xSupplier.get(), ySupplier.get(), turnSupplier.get());
        }));
        registerStateCommand(State.AUTO_INTAKE, new RunCommand(() -> {
            autoIntakeDrive();
            swerveDrive.periodic();
        }));
    }

    private void autoIntakeDrive() {

    }

    public enum State {
        UNDETERMINED,
        IDLE,
        TRAVERSING,
        AUTO_INTAKE
    }
}
