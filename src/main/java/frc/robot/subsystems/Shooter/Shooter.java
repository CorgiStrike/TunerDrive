package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Shooter.Arm.Arm;
import frc.robot.subsystems.Shooter.Arm.ArmIOReal;
import frc.robot.subsystems.Shooter.Flywheels.Flywheels;
import frc.robot.subsystems.Shooter.Flywheels.FlywheelsIOReal;
import frc.robot.util.WhileDisabledInstantCommand;

import static frc.robot.Constants.Flywheel.Settings.*;

public class Shooter extends StateMachine<Shooter.State> {

    private final Flywheels flywheels;
    private final Arm arm;

    private boolean doRapidSpinup = false;

    public Shooter(FlywheelsIOReal flywheelsIO, ArmIOReal armIO) {
        super("Shooter", State.UNDETERMINED, State.class);
        this.flywheels = new Flywheels(flywheelsIO);
        this.arm = new Arm(armIO, null, null);
        registerStateCommands();
        registerStateTransitions();

        addChildSubsystem(flywheels);
        addChildSubsystem(arm);
    }

    private Command watchReadyCommand() {
    return new RunCommand(
        () -> {
            if (arm.isFlag(Arm.State.AT_TARGET) && flywheels.isFlag(Flywheels.State.AT_SPEED)) {
                setFlag(State.READY);
            } else {
                clearFlag(State.READY);
            }
        });
    }

    public Command enableRapidSpinup() {
        return new WhileDisabledInstantCommand(
            () -> {
                doRapidSpinup = true;
            });
        }

    public Command disableRapidSpinup() {
        return new WhileDisabledInstantCommand(
            () -> {
                doRapidSpinup = false;
            });
    }

    private void registerStateCommands() {
        registerStateCommand(
        State.SOFT_E_STOP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.DISABLED),
            flywheels.transitionCommand(Flywheels.State.SOFT_E_STOP)));

        registerStateCommand(
            State.AMP,
            new ParallelCommandGroup(
                arm.transitionCommand(Arm.State.AMP),
                flywheels.transitionCommand(Flywheels.State.AMP),
                watchReadyCommand()));

        registerStateCommand(
            State.BASE_SHOT_SPEAKER,
            new ParallelCommandGroup(
                arm.transitionCommand(Arm.State.BASE_SHOT_SPEAKER),
                flywheels.transitionCommand(Flywheels.State.BASE_SHOT_SPEAKER),
                watchReadyCommand()));

        registerStateCommand(
            State.INTAKE,
            new ParallelCommandGroup(
                arm.transitionCommand(Arm.State.INTAKE),
                flywheels.transitionCommand(Flywheels.State.INTAKE),
                watchReadyCommand()));

        registerStateCommand(
            State.PASS_THROUGH,
            new SequentialCommandGroup(
                flywheels.transitionCommand(Flywheels.State.PASS_THROUGH),
                arm.transitionCommand(Arm.State.IDLE)));

        // This is where we do rapid spinup stuff in the auton mode where we blast the flywheels with
        // full power
        // just to get them up to speed
        registerStateCommand(
            State.SPEAKER_AA,
            new ParallelCommandGroup(
                new ConditionalCommand(
                    flywheels.transitionCommand(Flywheels.State.SPEAKER_AA),
                    new SequentialCommandGroup(
                        disableRapidSpinup(),
                        flywheels.transitionCommand(Flywheels.State.BASE_SHOT_SPEAKER),
                        new WaitUntilCommand(() -> flywheels.getCurrentTopSpeed() >= .90 * BASE_SHOT_VELOCITY)
                            .withTimeout(3),
                        flywheels.transitionCommand(Flywheels.State.SPEAKER_AA)),
                    () -> !doRapidSpinup),
                arm.transitionCommand(Arm.State.SPEAKER_AA),
                watchReadyCommand()));

        registerStateCommand(
            State.LOB_AA,
            new ParallelCommandGroup(
                flywheels.transitionCommand(Flywheels.State.LOB_AA),
                arm.transitionCommand(Arm.State.LOB_AA),
                watchReadyCommand()));

        registerStateCommand(
            State.LOB_STRAIGHT,
            new ParallelCommandGroup(
                flywheels.transitionCommand(Flywheels.State.LOB_STRAIGHT),
                arm.transitionCommand(Arm.State.LOB_STRAIGHT),
                watchReadyCommand()));

        registerStateCommand(
            State.LOB_ARC,
            new ParallelCommandGroup(
                flywheels.transitionCommand(Flywheels.State.LOB_ARC),
                arm.transitionCommand(Arm.State.LOB_ARC),
                watchReadyCommand()));
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.PARTIAL_SPINUP);
        addOmniTransition(State.INTAKE);
        addOmniTransition(State.SPEAKER_AA);
        addOmniTransition(State.BASE_SHOT_SPEAKER);
        addOmniTransition(State.AMP);
        addOmniTransition(State.LOB_AA);
        addOmniTransition(State.LOB_ARC);
        addOmniTransition(State.LOB_STRAIGHT);
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }
    
    public enum State {
        UNDETERMINED,
        SOFT_E_STOP,
        IDLE,
        PARTIAL_SPINUP,
        INTAKE,
        SPEAKER_AA,
        BASE_SHOT_SPEAKER,
        AMP,
        LOB_AA,
        LOB_ARC,
        LOB_STRAIGHT,
        PASS_THROUGH,

        // FLAGS
        READY
    }
}
