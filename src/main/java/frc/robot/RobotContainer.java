// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SMF.StateMachine;
import frc.robot.controllers.RealControllerBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOReal;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Arm.ArmIOReal;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOReal;
import frc.robot.util.AutoManager;


public class RobotContainer extends StateMachine<RobotContainer.State>{
  private RealControllerBindings controllerBindings = new RealControllerBindings();

  //initialize subsystems
  private final Intake intake;
  private final Indexer indexer;
  //private final Shooter shooter;

  private final BooleanSupplier flipPath = () ->{var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red & !DriverStation.isTeleop();
    }
    return false;
  };

  private final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
    TunerConstants.DrivetrainConstants, 
    TunerConstants.speedAt12VoltsMps, 
    Constants.Drivetrain.MAX_ANGULAR_RATE, 
    flipPath, 
    TunerConstants.FrontLeft, 
    TunerConstants.FrontRight, 
    TunerConstants.BackLeft, 
    TunerConstants.BackRight
  );

  private final Telemetry logger = new Telemetry(TunerConstants.speedAt12VoltsMps);

  private final AutoManager autoManager = new AutoManager();

  private void configureBindings() {
    drivetrain.configureBindings(controllerBindings::getDriveXValue, controllerBindings::getDriveYValue, controllerBindings::getDriveTurnValue);

    // reset the field-centric heading on right flight stick, left bumper press
    controllerBindings.resetGyro().onTrue(drivetrain.runOnce(() -> drivetrain.swerveDrive.seedFieldRelative()));

    //ground intake on A button
    controllerBindings.manualIntake()
    .onTrue(transitionCommand(State.GROUND_INTAKE, false))
    .onFalse(transitionCommand(State.TRAVERSING));

    controllerBindings.autoIntake()
    .onTrue(transitionCommand(State.AUTO_GROUND_INTAKE, false))
    .onFalse(transitionCommand(State.TRAVERSING));

    //ground eject on B button
    controllerBindings.intakeEject()
    .onTrue(transitionCommand(State.GROUND_EJECT, false))
    .onFalse(transitionCommand(State.TRAVERSING));
    
    controllerBindings.cleanseIndexer()
    .onTrue(transitionCommand(State.CLEANSE, false))
    .onFalse(transitionCommand(State.TRAVERSING));

    controllerBindings.stow()
    .onTrue(transitionCommand(State.TRAVERSING));

    controllerBindings.baseShot()
    .onTrue(transitionCommand(State.BASE_SHOT));

    controllerBindings.humanIntake()
    .onTrue(transitionCommand(State.HUMAN_INTAKE));

    controllerBindings.speakerAA()
    .onTrue(transitionCommand(State.SPEAKER_SCORE))
    .onFalse(transitionCommand(State.TRAVERSING));

    controllerBindings.lobAA()
    .onTrue(transitionCommand(State.LOB_AA))
    .onFalse(transitionCommand(State.TRAVERSING));

    if (Utils.isSimulation()) {
      drivetrain.swerveDrive.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.swerveDrive.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    super("RobotContainer", State.UNDETERMINED, State.class);
    
    //define subsystems
    intake = new Intake(new IntakeIOReal());
    indexer = new Indexer(new IndexerIOReal());
    /*shooter = new Shooter(
      new ArmIOReal(),
      new FlywheelIOReal(),
      () -> drivetrain.getPose().getTranslation()
    );*/

    // Add SMF Children
    addChildSubsystem(drivetrain);
    addChildSubsystem(intake);
    addChildSubsystem(indexer);
    //addChildSubsystem(shooter);

    configureBindings();
    registerStateTransitions();
    registerStateCommands();

    initializeShuffleboardTabs();
  }

  private void registerStateTransitions() {
    addTransition(State.TRAVERSING, State.AUTO_GROUND_INTAKE);
    addTransition(State.TRAVERSING, State.GROUND_INTAKE);
    addTransition(State.TRAVERSING, State.GROUND_EJECT);
    addTransition(State.TRAVERSING, State.AUTO_GROUND_INTAKE);
    addTransition(State.TRAVERSING, State.SPEAKER_SCORE);
    addTransition(State.TRAVERSING, State.LOB_AA);
    addOmniTransition(State.BASE_SHOT);
    addOmniTransition(State.HUMAN_INTAKE);

    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.TRAVERSING);
    addOmniTransition(State.LOST_NOTE);
    addOmniTransition(State.CLEANSE);
    addOmniTransition(State.AUTONOMOUS);
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, new ParallelCommandGroup(
      drivetrain.transitionCommand(CommandSwerveDrivetrain.State.IDLE),
      intake.transitionCommand(Intake.State.IDLE),
      indexer.transitionCommand(Indexer.State.SOFT_E_STOP)
      //shooter.transitionCommand(Shooter.State.SOFT_E_STOP)
    ));

    registerStateCommand(State.AUTONOMOUS, new PrintCommand("AUTO PLSS"));

    registerStateCommand(State.GROUND_INTAKE, new SequentialCommandGroup(
      new ParallelCommandGroup(
        drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
        intake.transitionCommand(Intake.State.INTAKING),
        indexer.transitionCommand(Indexer.State.AWAITING_NOTE_BACK)
      ),
      indexer.waitForState(Indexer.State.INDEXING),
      transitionCommand(State.TRAVERSING)));
    
    registerStateCommand(State.AUTO_GROUND_INTAKE, new SequentialCommandGroup(
      new ParallelCommandGroup(
        drivetrain.transitionCommand(CommandSwerveDrivetrain.State.AUTO_INTAKE),
        intake.transitionCommand(Intake.State.INTAKING),
        indexer.transitionCommand(Indexer.State.AWAITING_NOTE_BACK)),
      indexer.waitForState(Indexer.State.INDEXING),
      transitionCommand(State.TRAVERSING)));

    registerStateCommand(State.GROUND_EJECT, new ParallelCommandGroup(
        drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
        intake.transitionCommand(Intake.State.EJECTING),
        indexer.transitionCommand(Indexer.State.IDLE)
      ));
    
    registerStateCommand(State.TRAVERSING, 
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
        intake.transitionCommand(Intake.State.IDLE),
        indexer.transitionCommand(Indexer.State.IDLE)
        //shooter.transitionCommand(Shooter.State.TRAVERSING)
      ),
      new WaitUntilCommand(() -> indexer.getState() == Indexer.State.LOST_NOTE),
      transitionCommand(State.LOST_NOTE)                                                                                              
    ));

    registerStateCommand(State.CLEANSE, 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
          intake.transitionCommand(Intake.State.IDLE),
          indexer.transitionCommand(Indexer.State.PASS_THROUGH)
          //shooter.transitionCommand(Shooter.State.PASS_THROUGH)
        ),
        new WaitCommand(4),
        new ConditionalCommand(
          transitionCommand(State.TRAVERSING), 
          transitionCommand(State.LOST_NOTE), 
          () -> indexer.getState() == Indexer.State.IDLE)
      )
    );

    registerStateCommand(State.LOST_NOTE, 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          //set LEDs to an error color
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
          intake.transitionCommand(Intake.State.IDLE),
          indexer.transitionCommand(Indexer.State.IDLE)
        ),
        new WaitCommand(2.5), //provide the drivers some time to see the error LEDs
        transitionCommand(State.TRAVERSING)
      )
    );

    registerStateCommand(State.BASE_SHOT, 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
          intake.transitionCommand(Intake.State.IDLE),
          indexer.transitionCommand(Indexer.State.IDLE)
        ),
        waitReady(Shooter.State.BASE_SHOT)
      )
    );

    registerStateCommand(State.HUMAN_INTAKE,
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.SOURCE), //maybe change eventually to point towards
          indexer.transitionCommand(Indexer.State.AWAITING_NOTE_FRONT),
          //shooter.transitionCommand(Shooter.State.CHUTE_INTAKE),
          intake.transitionCommand(Intake.State.IDLE)
        ),
        new WaitUntilCommand(()-> indexer.getState() == Indexer.State.HAS_NOTE),
        transitionCommand(State.TRAVERSING)
      )
    );

    registerStateCommand(State.SPEAKER_SCORE,
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
          intake.transitionCommand(Intake.State.IDLE),
          indexer.transitionCommand(Indexer.State.IDLE)
        ),
        new WaitUntilCommand(() -> indexer.getState() == Indexer.State.HAS_NOTE),
      //shooter.transitionCommand(Shooter.State.SPEAKER_AA),
      //new WaitUntilCommand(() -> feedToShooter() && shooter.isFlag(Shooter.State.READY)),
      indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER),
      new WaitUntilCommand(() -> indexer.getState() == Indexer.State.IDLE || indexer.getState() == Indexer.State.LOST_NOTE),
      transitionCommand(State.TRAVERSING)
      )
    );

    registerStateCommand(State.LOB_AA,
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.LOB_AA),
          intake.transitionCommand(Intake.State.IDLE),
          indexer.transitionCommand(Indexer.State.IDLE)
        ),
        new WaitUntilCommand(() -> indexer.getState() == Indexer.State.HAS_NOTE),
        //shooter.transitionCommand(Shooter.State.LOB_ACTIVE_ADJUST),
        //new WaitUntilCommand(() -> feedToShooter() && shooter.isFlag(Shooter.State.READY)),
        indexer.transitionCommand(Indexer.State.FEED_TO_SHOOTER),
        new WaitUntilCommand(() -> indexer.getState() == Indexer.State.IDLE || indexer.getState() == Indexer.State.LOST_NOTE),
        transitionCommand(State.TRAVERSING)
      )
    );
  }


  private boolean feedToShooter(){
    return controllerBindings.feedShooter().getAsBoolean();
  }

  private void initializeShuffleboardTabs(){
    ShuffleboardTab teleTab = Shuffleboard.getTab("Tele");

    teleTab
      .addString("Indexer State", () -> indexer.getState().toString())
      .withPosition(3, 3)
      .withSize(2,1);
  }

  private Command waitReady(Shooter.State state){
    return new SequentialCommandGroup(
      
    );
  }

  @Override
  protected void determineSelf() {
    setState(State.TRAVERSING);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVERSING);
  }

  @Override
  protected void onAutonomousStart() {
    drivetrain.setAutoCommand(autoManager.getSelectedCommand());
    drivetrain.requestTransition(CommandSwerveDrivetrain.State.FOLLOWING_AUTONOMOUS_COMMAND);
  }

  @Override
  protected void update() {
    
  }

  public enum State {
    UNDETERMINED,
    AUTONOMOUS,
    SOFT_E_STOP,
    TRAVERSING,
    AUTO_GROUND_INTAKE,
    GROUND_INTAKE,
    GROUND_EJECT,
    LOST_NOTE,
    CLEANSE,
    BASE_SHOT,
    HUMAN_INTAKE,
    SPEAKER_SCORE,
    LOB_AA
  }
}
