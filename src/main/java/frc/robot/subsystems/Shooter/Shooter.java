package frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.Flywheel.Settings.BASE_SHOT_VELOCITY;
import static frc.robot.Constants.Shooter.Settings.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.util.AllianceManager;
import frc.robot.SMF.StateMachine;
import frc.robot.util.WhileDisabledInstantCommand;
import frc.robot.subsystems.Shooter.Arm.Arm;
import frc.robot.subsystems.Shooter.Arm.ArmIO;
import frc.robot.subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends StateMachine<Shooter.State> {
  private final Arm arm;
  private final Flywheel flywheel;

  // odom
  private final Supplier<Translation2d> botTranslationProvider;

  private boolean doRapidSpinup = false;

  public Shooter(
      ArmIO armIO,
      FlywheelIO flywheelIO,
      Supplier<Translation2d> botTranslationProvider) {
    super("Shooter", State.UNDETERMINED, State.class);

    this.botTranslationProvider = botTranslationProvider;

    arm =
        new Arm(
            armIO,
            this::armSpeakerAA,
            this::armLobAA
        );

    flywheel =
        new Flywheel(
            flywheelIO,
            this::flywheelSpeakerAA,
            this::flywheelLobAA
            );

    addChildSubsystem(arm);
    addChildSubsystem(flywheel);

    registerStateCommands();
    registerTransitions();

    SmartDashboard.putData(arm);
    SmartDashboard.putData(flywheel);
  }

  private void registerStateCommands() {
    registerStateCommand(
        State.SOFT_E_STOP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.SOFT_E_STOP),
            flywheel.transitionCommand(Flywheel.State.IDLE)));

    registerStateCommand(
        State.AMP,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.AMP),
            flywheel.transitionCommand(Flywheel.State.AMP),
            watchReadyCommand()));

    registerStateCommand(
        State.BASE_SHOT,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.BASE_SHOT),
            flywheel.transitionCommand(Flywheel.State.BASE_SHOT_SPIN),
            watchReadyCommand()));

    registerStateCommand(
        State.AUTO_START_SHOT,
        new ParallelCommandGroup(
            flywheel.transitionCommand(Flywheel.State.BASE_SHOT_SPIN),
            arm.transitionCommand(Arm.State.AUTO_START_SHOT),
            watchReadyCommand()));

    registerStateCommand(
        State.CHUTE_INTAKE,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.CHUTE_INTAKE),
            flywheel.transitionCommand(Flywheel.State.CHUTE_INTAKE),
            watchReadyCommand()));

    registerStateCommand(
        State.STOW,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.FULL_STOW),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            watchReadyCommand()));

    registerStateCommand(
        State.PARTIAL_STOW,
        new ParallelCommandGroup(
            arm.transitionCommand(Arm.State.PARTIAL_STOW),
            flywheel.transitionCommand(Flywheel.State.IDLE),
            watchReadyCommand()));

    registerStateCommand(
        State.PASS_THROUGH,
        new SequentialCommandGroup(
            flywheel.transitionCommand(Flywheel.State.PASS_THROUGH),
            arm.transitionCommand(Arm.State.PARTIAL_STOW)));
      
    registerStateCommand(State.TRAVERSING,
     new SequentialCommandGroup(
      partialFlywheelSpinup(),
      arm.transitionCommand(Arm.State.FULL_STOW)
     ));

    // This is where we do rapid spinup stuff in the auton mode where we blast the flywheels with
    // full power
    // just to get them up to speed
    registerStateCommand(
        State.SPEAKER_AA,
        new ParallelCommandGroup(
            flywheel.transitionCommand(Flywheel.State.SPEAKER_ACTIVE_ADJUST_SPIN),
            arm.transitionCommand(Arm.State.SHOT_ACTIVE_ADJUST),
            watchReadyCommand()
            )
          );

    registerStateCommand(
        State.LOB_ACTIVE_ADJUST,
        new ParallelCommandGroup(
            flywheel.transitionCommand(Flywheel.State.LOB_ACTIVE_ADJUST),
            arm.transitionCommand(Arm.State.LOB_ACTIVE_ADJUST),
            watchReadyCommand()));

    registerStateCommand(
        State.LOB_STRAIGHT,
        new ParallelCommandGroup(
            flywheel.transitionCommand(Flywheel.State.LOB_STRAIGHT),
            arm.transitionCommand(Arm.State.LOB_STRAIGHT),
            watchReadyCommand()));

    registerStateCommand(
        State.LOB_ARC,
        new ParallelCommandGroup(
            flywheel.transitionCommand(Flywheel.State.LOB_ARC),
            arm.transitionCommand(Arm.State.LOB_ARC),
            watchReadyCommand()));
  }

  public Command enableRapidSpinup() {
    return new WhileDisabledInstantCommand(
        () -> {
          doRapidSpinup = true;
        });
  }

  public Command disableRapidSpinup() {
    return new WhileDisabledInstantCommand(
        () -> {
          doRapidSpinup = false;
        });
  }

  private void registerTransitions() {
    // omnis cause none of these states conflict with anything within the subsystem
    addOmniTransition(State.SOFT_E_STOP);

    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.AUTO_START_SHOT);
    addOmniTransition(State.STOW);
    addOmniTransition(State.PARTIAL_STOW);
    addOmniTransition(State.CHUTE_INTAKE);
    addOmniTransition(State.AMP);
    addOmniTransition(State.PASS_THROUGH);
    addOmniTransition(State.SPEAKER_AA);
    addOmniTransition(State.LOB_STRAIGHT);
    addOmniTransition(State.LOB_ARC);
    addOmniTransition(State.MOVING_SPEAKER_AA);
    addOmniTransition(State.LOB_ACTIVE_ADJUST);
    addOmniTransition(State.TUNE);
    addOmniTransition(State.TRAVERSING);

    addTransition(State.SOFT_E_STOP, State.BOTTOM_FLYWHEEL_VOLTAGE_CALC);
    addTransition(State.SOFT_E_STOP, State.FLYWHEEL_VOLTAGE_CALC);
    addTransition(State.SOFT_E_STOP, State.ARM_VOLTAGE_CALC);
  }

  public Command syncAbsoluteAngle() {
    return new WhileDisabledInstantCommand(
        () -> {
          arm.syncAbsoluteAngle();
        });
  }

  public Command partialFlywheelSpinup() {
    return flywheel.transitionCommand(Flywheel.State.PARTIAL_SPINUP);
  }

  public Command flywheelSpinDown() {
    return flywheel.transitionCommand(Flywheel.State.IDLE);
  }

  public Command indicateAmpIntention() {
    return flywheel.transitionCommand(Flywheel.State.AMP);
  }

  private Command watchReadyCommand() {
    return new RunCommand(
        () -> {
          if (arm.isFlag(Arm.State.AT_TARGET) && flywheel.isFlag(Flywheel.State.AT_SPEED)) {
            setFlag(State.READY);
          } else {
            clearFlag(State.READY);
          }
        });
  }

  private double armLobAA() {
    double distance = getCornerDistance();

    return ARM_LOB_DISTANCE_LUT.get(distance);
  }

  private double flywheelLobAA() {
    double distance = getCornerDistance();

    return FLYWHEEL_LOB_DISTANCE_LUT.get(distance);
  }

  private double armSpeakerAA() {
    double distance = getSpeakerDistance();

    // angle of shooter to face directly at speaker target height plus lut offset
    return Math.atan2(SPEAKER_TARGET_HEIGHT, distance) + SPEAKER_LUT.get(distance);
  }


  private double flywheelSpeakerAA() {
    return FLYWHEEL_SPEAKER_DISTANCE_LUT.get(getSpeakerDistance());
  }

  @AutoLogOutput(key = "Shooter/SpeakerDistance")
  private double getSpeakerDistance() {
    Pose2d speaker =
        AllianceManager.getAlliance() == DriverStation.Alliance.Blue
            ? Constants.PhysicalConstants.BLUE_SPEAKER
            : Constants.mirror(Constants.PhysicalConstants.BLUE_SPEAKER);

    return speaker.getTranslation().getDistance(botTranslationProvider.get());
  }

  @AutoLogOutput(key = "Shooter/CornerDistance")
  private double getCornerDistance() {
    Pose2d corner =
        AllianceManager.getAlliance() == DriverStation.Alliance.Blue
            ? Constants.PhysicalConstants.BLUE_CORNER
            : Constants.mirror(Constants.PhysicalConstants.BLUE_LOB_CORNER);

    return corner.getTranslation().getDistance(botTranslationProvider.get());
  }

  public double getArmAngle() {
    return arm.getAngle();
  }

  public double getArmAbsoluteAngle() {
    return arm.getAbsoluteAngle();
  }

  @Override
  protected void determineSelf() {
    // await instructions from rc
    setState(State.SOFT_E_STOP);
  }

  @Override
  protected void update(){
    //new PrintCommand(getState().name()).schedule();
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    BASE_SHOT,
    AUTO_START_SHOT,
    STOW,
    PARTIAL_STOW,
    CHUTE_INTAKE,
    FLYWHEEL_VOLTAGE_CALC,
    BOTTOM_FLYWHEEL_VOLTAGE_CALC,
    ARM_VOLTAGE_CALC,
    AMP,
    PASS_THROUGH,
    SPEAKER_AA,
    LOB_STRAIGHT,
    LOB_ARC,
    MOVING_SPEAKER_AA,
    LOB_ACTIVE_ADJUST,
    TUNE,
    TRAVERSING,
    // flags
    READY, state
  }
}
