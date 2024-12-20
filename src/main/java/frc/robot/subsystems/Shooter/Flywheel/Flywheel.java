package frc.robot.subsystems.Shooter.Flywheel;

import static frc.robot.Constants.Shooter.Flywheel.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;
import java.util.function.DoubleSupplier;

public class Flywheel extends StateMachine<Flywheel.State> {
  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();

  private final DoubleSupplier speakerAAProvider;
  private final DoubleSupplier lobAASupplier;

  public Flywheel(
      FlywheelIO io,
      DoubleSupplier speakerAAProvider,
      DoubleSupplier lobAASupplier) {
    super("Shooter Flywheel", State.UNDETERMINED, State.class);

    this.speakerAAProvider = speakerAAProvider;
    this.lobAASupplier = lobAASupplier;
    this.io = io;

    registerStateCommands();
    registerTransitions();

    SmartDashboard.putData("flywheel", this);
  }

  private void registerStateCommands() {

    registerStateCommand(
        State.BASE_SHOT_SPIN,
        new ParallelCommandGroup(
            new InstantCommand(() -> io.setFlywheelTarget(BASE_SHOT_VELOCITY)),
            atSpeedCommand(() -> BASE_SHOT_VELOCITY, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(State.IDLE, io::stop);

    registerStateCommand(
        State.SPEAKER_ACTIVE_ADJUST_SPIN,
        new ParallelCommandGroup(
            new RunCommand(() -> io.setFlywheelTarget(speakerAAProvider.getAsDouble())),
            atSpeedCommand(speakerAAProvider, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(
        State.LOB_ACTIVE_ADJUST,
        new ParallelCommandGroup(
            new RunCommand(() -> io.setFlywheelTarget(lobAASupplier.getAsDouble())),
            atSpeedCommand(lobAASupplier, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(State.PASS_THROUGH, () -> io.setFlywheelTarget(PASS_THROUGH_SPEED));

    registerStateCommand(State.CHUTE_INTAKE, () -> io.setFlywheelTarget(CHUTE_INTAKE_SPEED));

    registerStateCommand(
        State.AMP,
        new ParallelCommandGroup(
            new InstantCommand(() -> io.setFlywheelTargets(AMP_SPEED_TOP, AMP_SPEED_BOTTOM)),
            atSpeedCommand(() -> AMP_SPEED_TOP, SPIN_UP_READY_TOLERANCE)));

    registerStateCommand(
        State.PARTIAL_SPINUP,
        new InstantCommand(() -> io.setFlywheelTarget(PARTIAL_SPINUP_VELOCITY)));

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

    registerStateCommand(
        State.FULL_POWER,
        new InstantCommand(
            () -> {
              io.setDutyCycle(1);
            }));
  }

  private void registerTransitions() {
    // omni everything cause flywheel spinning around isnt gonna hurt anything
    addOmniTransition(State.IDLE);
    addOmniTransition(State.BASE_SHOT_SPIN);
    addOmniTransition(State.SPEAKER_ACTIVE_ADJUST_SPIN);
    addOmniTransition(State.PASS_THROUGH);
    addOmniTransition(State.CHUTE_INTAKE);
    addOmniTransition(State.AMP);
    addOmniTransition(State.PARTIAL_SPINUP);
    addOmniTransition(State.LOB_STRAIGHT);
    addOmniTransition(State.LOB_ARC);
    addOmniTransition(State.FULL_POWER);
    addOmniTransition(State.LOB_ACTIVE_ADJUST);
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
    BASE_SHOT_SPIN,
    IDLE,
    SPEAKER_ACTIVE_ADJUST_SPIN,
    PASS_THROUGH,
    CHUTE_INTAKE,
    AMP,
    PARTIAL_SPINUP,
    LOB_STRAIGHT,
    LOB_ARC,
    FULL_POWER,
    LOB_ACTIVE_ADJUST,

    // flags
    AT_SPEED
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }

}
