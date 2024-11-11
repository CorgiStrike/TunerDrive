package frc.robot.subsystems.Shooter.Arm;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Arm.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SMF.StateMachine;

public class Arm extends StateMachine<Arm.State> {
    private final ArmIOReal io;
    
    private final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    private final DoubleSupplier lobAASupplier;
    private final DoubleSupplier speakerAAProvider;

    public Arm(ArmIOReal io, DoubleSupplier lobAASupplier, DoubleSupplier speakerAAProvider) {
        super("Arm", State.UNDETERMINED, State.class);
        this.io = io;
        this.lobAASupplier = lobAASupplier;
        this.speakerAAProvider = speakerAAProvider;
        registerStateCommands();
        registerStateTransitions();
    }

    private void registerStateCommands() {
        registerStateCommand(State.DISABLED, io::stop);

        registerStateCommand(State.AMP, holdPositionCommand(() -> AMP_POSITION));
        registerStateCommand(State.BASE_SHOT_SPEAKER, holdPositionCommand(() -> BASE_SHOT_POSITION));
        registerStateCommand(State.INTAKE, holdPositionCommand(() -> CHUTE_INTAKE_POSITION));

        registerStateCommand(State.LOB_AA, holdPositionCommand(lobAASupplier));

        registerStateCommand(State.LOB_STRAIGHT, holdPositionCommand(() -> LOB_POSITION_STRAIGHT));
        registerStateCommand(State.LOB_ARC, holdPositionCommand(() -> LOB_POSITION_ARC));

        registerStateCommand(State.SPEAKER_AA, holdPositionCommand(speakerAAProvider));
    }

    private void registerStateTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.DISABLED);
        addOmniTransition(State.INTAKE);
        addOmniTransition(State.LOB_AA);
        addOmniTransition(State.SPEAKER_AA);
        addOmniTransition(State.AMP);
        addOmniTransition(State.CLEANSE);
        addOmniTransition(State.BASE_SHOT_SPEAKER);
        addOmniTransition(State.LOB_ARC);
        addOmniTransition(State.LOB_STRAIGHT);
    }

    private Command holdPositionCommand(DoubleSupplier positionProvider) {
    return new ParallelCommandGroup(
        new RunCommand(
            () -> {
                double target = positionProvider.getAsDouble();

              // avoid spamming can network :)
                if (!doubleEqual(target, inputs.targetPosition)) {
                    io.setTargetPosition(target);
                }
            }),
        atTargetCommand(positionProvider));
    }

    private Command atTargetCommand(DoubleSupplier positionProvider) {
        return new RunCommand(
            () -> {
                if (doubleEqual(
                    inputs.motorPosition, positionProvider.getAsDouble(), POSITION_READY_TOLERANCE)) {
                    setFlag(State.AT_TARGET);
                } else {
                    clearFlag(State.AT_TARGET);
                }
                
            });
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    @Override
    protected void update() {
        io.updateInputs(inputs);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        DISABLED,
        INTAKE,
        LOB_AA,
        SPEAKER_AA,
        AMP,
        CLEANSE,
        BASE_SHOT_SPEAKER,
        LOB_ARC,
        LOB_STRAIGHT,

        // flags
        AT_TARGET
    }
}
