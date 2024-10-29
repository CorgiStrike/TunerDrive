package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.Intake.Hardware;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Intake.IntakeIO.*;

public class Intake extends StateMachine<Intake.State> {
    private final IntakeIO io;

    private final IntakeIOInputs inputs = new IntakeIOInputs();

    //ran when initializing the intake
    public Intake (IntakeIO io){
        super("Intake", State.UNDETERMINED, State.class);
        this.io = io;

        io.resetFollower();

        io.updateInputs(inputs);

        registerStateCommands();
        registerTransitions();
    }

    //define what the intake does in each state
    public void registerStateCommands() {
        registerStateCommand(State.IDLE, new InstantCommand(io::stop));
        registerStateCommand(State.EJECTING, new InstantCommand(() -> io.setBeltTargetVelocity(-Hardware.BELT_SPEED)));
        registerStateCommand(State.INTAKING, new SequentialCommandGroup(
            new InstantCommand(() -> io.setBeltTargetVelocity(Hardware.BELT_SPEED)),
            new WaitCommand(1),
            new WaitUntilCommand(() -> inputs.velocity < 0.15 * inputs.targetVelocity),
            new InstantCommand(() -> io.setBeltTargetVelocity(0)),
            new WaitCommand(0.1)
        ).repeatedly()
        );
    }

    //define the transitions for the intake
    public void registerTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.INTAKING);
        addOmniTransition(State.EJECTING);
    }

    //check if the ring is present using the prox sensor
    public boolean ringPresent() {
        return inputs.proxTripped;
    }

    //override the super to always know the state of the intake
    @Override
    protected void determineSelf() {
        io.resetFollower();
        setState(State.IDLE);
    }

    //override the super to constantly update the inputs
    @Override
    protected void update() {
        io.updateInputs(inputs);
    }

    //define the states for the intake
    public enum State {
        UNDETERMINED,
        IDLE,
        INTAKING,
        EJECTING
    }
}