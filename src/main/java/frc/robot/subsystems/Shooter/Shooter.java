package frc.robot.subsystems.Shooter;

import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Shooter.Flywheels.Flywheels;
import frc.robot.subsystems.Shooter.Flywheels.FlywheelsIOReal;

public class Shooter extends StateMachine<Shooter.State> {

    private final Flywheels flywheels;

    public Shooter(FlywheelsIOReal flywheelsIO) {
        super("Shooter", State.UNDETERMINED, State.class);
        this.flywheels = new Flywheels(flywheelsIO);
        registerStateCommands();
        registerStateTransitions();

        addChildSubsystem(flywheels);
    }

    /*private Command watchReadyCommand() {
    return new RunCommand(
        () -> {
            if (arm.isFlag(Arm.State.AT_TARGET) && flywheel.isFlag(Flywheel.State.AT_SPEED)) {
                setFlag(State.READY);
            } else {
                clearFlag(State.READY);
            }
        });
    }*/

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, flywheels.transitionCommand(Flywheels.State.IDLE));
        registerStateCommand(State.PARTIAL_SPINUP, flywheels.transitionCommand(Flywheels.State.PARTIAL_SPINUP));
        registerStateCommand(State.INTAKE, flywheels.transitionCommand(Flywheels.State.INTAKE));
        registerStateCommand(State.SPEAKER_AA, flywheels.transitionCommand(Flywheels.State.SPEAKER_AA));
        registerStateCommand(State.BASE_SHOT_SPEAKER, flywheels.transitionCommand(Flywheels.State.BASE_SHOT_SPEAKER));
        registerStateCommand(State.AMP, flywheels.transitionCommand(Flywheels.State.AMP));
        registerStateCommand(State.LOB_AA, flywheels.transitionCommand(Flywheels.State.LOB_AA));
        registerStateCommand(State.LOB_ARC, flywheels.transitionCommand(Flywheels.State.LOB_ARC));
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
        AT_SPEED
    }
}
