package frc.robot.subsystems.Shooter.Flywheels;

import frc.robot.SMF.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.doubleEqual;
import static frc.robot.Constants.Flywheel.Settings.*;

import java.util.function.DoubleSupplier;

public class Flywheels extends StateMachine<Flywheels.State> {
    private FlywheelsIOReal io;

    private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
    
    public Flywheels(FlywheelsIOReal io) {
        super("Flywheels",State.UNDETERMINED, State.class);
        this.io = io;
        registerStateCommands();
        registerStateTransitions();
    }

    private void registerStateCommands() {
        registerStateCommand(State.IDLE, () -> {
            io.stop();
        });

        registerStateCommand(
            State.AMP,
            new ParallelCommandGroup(
                new InstantCommand(() -> io.setFlywheelTargets(AMP_SPEED_TOP, AMP_SPEED_BOTTOM)),
                atSpeedCommand(() -> AMP_SPEED_TOP, SPIN_UP_READY_TOLERANCE)));

        registerStateCommand(
            State.BASE_SHOT_SPEAKER,
            new ParallelCommandGroup(
                new InstantCommand(() -> io.setFlywheelTarget(BASE_SHOT_VELOCITY)),
                atSpeedCommand(() -> BASE_SHOT_VELOCITY, SPIN_UP_READY_TOLERANCE)));
        
        registerStateCommand(State.PASS_THROUGH, () -> io.setFlywheelTarget(PASS_THROUGH_SPEED));

        registerStateCommand(State.INTAKE, () -> io.setFlywheelTarget(CHUTE_INTAKE_SPEED));

        registerStateCommand(
        State.LOB_STRAIGHT,
        new ParallelCommandGroup(
            new InstantCommand(
                () -> io.setFlywheelTargets(LOB_SPEED_STRAIGHT_TOP, LOB_SPEED_STRAIGHT_BOTTOM)),
            atSpeedCommand(() -> LOB_SPEED_STRAIGHT_TOP, SPIN_UP_READY_TOLERANCE)));

        registerStateCommand(
            State.LOB_ARC,
            new ParallelCommandGroup(
                new InstantCommand(() -> io.setFlywheelTarget(LOB_SPEED_ARC)),
                atSpeedCommand(() -> LOB_SPEED_ARC, SPIN_UP_READY_TOLERANCE)));
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

    private Command atSpeedCommand(DoubleSupplier speedProvider, double accuracy) {
    return new RunCommand(
        () -> {
            if (doubleEqual(inputs.velocity, speedProvider.getAsDouble(), accuracy)) {
                setFlag(State.AT_SPEED);
            } else {
                clearFlag(State.AT_SPEED);
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
        // await control from shooter
        io.resetFollower();
        setState(State.IDLE);
    }

    public double getCurrentTopSpeed() {
        return inputs.velocity;
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
