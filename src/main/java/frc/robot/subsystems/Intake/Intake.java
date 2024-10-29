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

    public Intake (IntakeIO io){
        super("Intake", State.UNDETERMINED, State.class);
        this.io = io;

        io.resetFollower();

        io.updateInputs(inputs);

        registerStateCommands();
        registerTransitions();
    }


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

    public void registerTransitions() {
        addOmniTransition(State.IDLE);
        addOmniTransition(State.INTAKING);
        addOmniTransition(State.EJECTING);
    }

    public boolean ringPresen() {
        return inputs.proxTripped;
    }

    @Override
    protected void determineSelf() {
        io.resetFollower();
        setState(State.IDLE);
    }

    @Override
    protected void update() {
        io.updateInputs(inputs);
    }

    public enum State {
        UNDETERMINED,
        IDLE,
        INTAKING,
        EJECTING
    }
}