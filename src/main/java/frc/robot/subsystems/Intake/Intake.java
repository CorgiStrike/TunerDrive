package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SMF.StateMachine;

import frc.robot.Constants.Intake.Hardware;

public class Intake extends StateMachine<Intake.State> {

    private final IntakeIO io = new IntakeIOReal();

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

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
        registerStateCommand(State.IDLE, new SequentialCommandGroup(new InstantCommand(io::stop), watchProxCommand()));

        registerStateCommand(State.INTAKE, 
        new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.BELT_SPEED)),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    
                ),


                watchProxCommand())
        ));        
    }

    private Command watchProxCommand(){
        return new RunCommand(
            () -> {if (inputs.proxTripped){
                setFlag(State.PROX_TRIPPED);
            }else{
                clearFlag(State.PROX_TRIPPED);
            }
            });
    }

    @Override
    protected void determineSelf() {
        setState(State.IDLE);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        INTAKE,
        EJECT,

        //flag
        PROX_TRIPPED
    }
}
