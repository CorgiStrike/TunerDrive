package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SMF.StateMachine;
import frc.robot.WhileDisabledInstantCommand;

import frc.robot.Constants.Intake.Hardware;

public class Intake extends StateMachine<Intake.State> {

    private final IntakeIO io;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        super("Intake", State.UNDETERMINED, State.class);

        this.io = io;

        new WaitCommand(2)
        .andThen(new WhileDisabledInstantCommand(() -> io.resetFollower()))
        .schedule();

        io.updateInputs(inputs);

        registerStateCommands();
        registerStateTransitions();
    }

    private void registerStateTransitions() {
        addOmniTransition(State.EJECT);
        addOmniTransition(State.INTAKE);
        addOmniTransition(State.IDLE);
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, new SequentialCommandGroup(new InstantCommand(io::stop), watchProxCommand()));

        registerStateCommand(
            State.INTAKE, 
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.BELT_SPEED)),
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        new WaitCommand(1),
                        new WaitUntilCommand(() -> inputs.velocity < 0.15 * inputs.targetVelocity),
                        new InstantCommand(() -> io.setBeltTargetVelocity(0)),
                        new WaitCommand(0.5),
                         new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.BELT_SPEED)))
                    .repeatedly(),
                watchProxCommand())
            ));        

        registerStateCommand(
            State.EJECT,
            new SequentialCommandGroup(
                new InstantCommand(() -> io.setBeltTargetVelocity(-Hardware.BELT_SPEED)), watchProxCommand()));
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
    protected void update() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
    }

    @Override
    protected void determineSelf() {
        io.resetFollower();
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
