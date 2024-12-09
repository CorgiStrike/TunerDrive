package frc.robot.subsystems.Shooter.Arm;

import static frc.robot.Constants.Shooter.Arm.Settings.*;
import static frc.robot.Constants.doubleEqual;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.subsystems.Shooter.Arm.ArmIO.ArmIOInputs;
import java.util.function.DoubleSupplier;

public class Arm extends StateMachine<Arm.State> {
  private final ArmIO io;
  private final ArmIOInputs inputs = new ArmIOInputs();

  // AA stands for active adjust
  private final DoubleSupplier distanceAAProvider;
  private final DoubleSupplier lobAASupplier;

  public Arm(
      ArmIO io,
      DoubleSupplier distanceAAProvider,
      DoubleSupplier lobAASupplier) {
    super("Shooter Arm", State.UNDETERMINED, State.class);

    this.io = io;
    this.distanceAAProvider = distanceAAProvider;
    this.lobAASupplier = lobAASupplier;

    registerStateCommands();
    registerTransitions();
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, io::stop);

    registerStateCommand(State.AMP, holdPositionCommand(() -> AMP_POSITION));
    registerStateCommand(State.BASE_SHOT, holdPositionCommand(() -> BASE_SHOT_POSITION));
    registerStateCommand(State.AUTO_START_SHOT, holdPositionCommand(() -> AUTO_START_POSITION));
    registerStateCommand(State.CHUTE_INTAKE, holdPositionCommand(() -> CHUTE_INTAKE_POSITION));
    registerStateCommand(State.PARTIAL_STOW, holdPositionCommand(() -> PARTIAL_STOW_POSITION));
    registerStateCommand(State.FULL_STOW, holdPositionCommand(() -> FULL_STOW_POSITION));

    registerStateCommand(State.LOB_ACTIVE_ADJUST, holdPositionCommand(lobAASupplier));

    registerStateCommand(State.LOB_STRAIGHT, holdPositionCommand(() -> LOB_POSITION_STRAIGHT));
    registerStateCommand(State.LOB_ARC, holdPositionCommand(() -> LOB_POSITION_ARC));

    registerStateCommand(State.SHOT_ACTIVE_ADJUST, holdPositionCommand(distanceAAProvider));
  }

  private void registerTransitions() {
    addOmniTransition(State.SOFT_E_STOP);

    // it going from one to the other wont conflict with anything within the arm subsystem
    addOmniTransition(State.AMP);
    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.AUTO_START_SHOT);
    addOmniTransition(State.CHUTE_INTAKE);
    addOmniTransition(State.PARTIAL_STOW);
    addOmniTransition(State.FULL_STOW);
    addOmniTransition(State.LOB_STRAIGHT);
    addOmniTransition(State.LOB_ARC);
    addOmniTransition(State.SHOT_ACTIVE_ADJUST);
    addOmniTransition(State.MOVING_SHOT_ACTIVE_ADJUST);
    addOmniTransition(State.LOB_ACTIVE_ADJUST);
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
  protected void update() {
    io.updateInputs(inputs);
  }

  @Override
  protected void determineSelf() {
    io.resetFollower();
    setState(State.SOFT_E_STOP);
  }

  public double getAngle() {
    return inputs.motorPosition;
  }

  public double getAbsoluteAngle() {
    return inputs.encoderPosition;
  }

  public void syncAbsoluteAngle() {
    io.syncToAbsoluteEncoder();
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    AMP,
    CHUTE_INTAKE,
    BASE_SHOT,
    AUTO_START_SHOT,
    SHOT_ACTIVE_ADJUST,
    PARTIAL_STOW,
    FULL_STOW,
    LOB_STRAIGHT,
    LOB_ARC,
    MOVING_SHOT_ACTIVE_ADJUST,
    LOB_ACTIVE_ADJUST,
    // flags
    AT_TARGET
  }
}

