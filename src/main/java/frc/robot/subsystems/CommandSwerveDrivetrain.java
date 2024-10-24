package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.SwerveDrive;
import frc.robot.SMF.StateMachine;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends StateMachine<CommandSwerveDrivetrain.State> implements Subsystem {
    public final SwerveDrive swerveDrive;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super("CommandSwerveDrive", State.UNDETERMINED, State.class);
        swerveDrive = new SwerveDrive(driveTrainConstants, OdometryUpdateFrequency, modules);
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super("RobotContainer", State.UNDETERMINED, State.class);
        swerveDrive = new SwerveDrive(driveTrainConstants, modules);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(swerveDrive.getRequestRunnable(requestSupplier));
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new RunCommand(() -> swerveDrive.periodic()));
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        TRAVERSING
    }
}
