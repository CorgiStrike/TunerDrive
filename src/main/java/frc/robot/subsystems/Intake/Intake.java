package frc.robot.subsystems.Intake;

import frc.robot.SMF.StateMachine;

public class Intake extends StateMachine<Intake.State> {


    public Intake() {
        super("Intake", State.UNDETERMINED, State.class);
        registerStateTransitions();
        registerStateCommands();
    }

    private void registerStateTransitions() {
        addOmniTransition(State.EJECT);
        addOmniTransition(State.INTAKE);
        addOmniTransition(State.IDLE);
    }

    private void registerStateCommands() {
        
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        INTAKE,
        EJECT
    }
}
