// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.SMF.StateMachine;
import frc.robot.controllers.RealControllerBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Flywheels.FlywheelsIOReal;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOReal;


public class RobotContainer extends StateMachine<RobotContainer.State>{
  private RealControllerBindings controllerBindings = new RealControllerBindings();

  //initialize subsystems
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

  private void configureBindings() {
    drivetrain.configureBindings(controllerBindings::getDriveXValue, controllerBindings::getDriveYValue, controllerBindings::getDriveTurnValue);

    // reset the field-centric heading on left bumper press
    controllerBindings.resetGyro().onTrue(drivetrain.runOnce(() -> drivetrain.swerveDrive.seedFieldRelative()));

    //ground intake on A button
    controllerBindings.manualIntake()
    .onTrue(transitionCommand(State.GROUND_INTAKE, false))
    .onFalse(
            new ConditionalCommand(
                transitionCommand(State.TRAVERSING, false),
                Commands.none(),
                () -> getState() == State.GROUND_INTAKE));

    controllerBindings.autoIntake()
    .onTrue(transitionCommand(State.AUTO_GROUND_INTAKE, false))
    .onFalse(
            new ConditionalCommand(
                transitionCommand(State.TRAVERSING, false),
                Commands.none(),
                () -> getState() == State.AUTO_GROUND_INTAKE));

    //ground eject on B button
    controllerBindings.intakeEject()
    .onTrue(transitionCommand(State.GROUND_EJECT, false))
    .onFalse(
      new ConditionalCommand(
          transitionCommand(State.TRAVERSING, false),
          Commands.none(),
          () -> getState() == State.GROUND_EJECT));

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
    shooter = new Shooter(new FlywheelsIOReal());

    // Add SMF Children
    addChildSubsystem(drivetrain);
    addChildSubsystem(intake);
    addChildSubsystem(indexer);

    configureBindings();
    registerStateTransitions();
    registerStateCommands();
  }

  private void registerStateTransitions() {
    addTransition(State.TRAVERSING, State.AUTO_GROUND_INTAKE);
    addTransition(State.TRAVERSING, State.GROUND_INTAKE);
    addTransition(State.TRAVERSING, State.GROUND_EJECT);
    addTransition(State.TRAVERSING, State.BASE_SHOT_SPEAKER);
    addTransition(State.TRAVERSING, State.LOB_ARC);
    addTransition(State.TRAVERSING, State.LOB_STRAIGHT);
    addTransition(State.TRAVERSING, State.SPEAKER_AA);
    addTransition(State.TRAVERSING, State.LOB_AA);

    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.TRAVERSING);
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, new ParallelCommandGroup(
      drivetrain.transitionCommand(CommandSwerveDrivetrain.State.IDLE),
      intake.transitionCommand(Intake.State.IDLE),
      indexer.transitionCommand(Indexer.State.SOFT_E_STOP)
    ));

    registerStateCommand(State.GROUND_INTAKE, new SequentialCommandGroup(
      new ParallelCommandGroup(
        drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
        intake.transitionCommand(Intake.State.INTAKING),
        indexer.transitionCommand(Indexer.State.INDEXING)
      ),
      indexer.waitForState(Indexer.State.HAS_NOTE),
      transitionCommand(State.TRAVERSING)));
    
    registerStateCommand(State.AUTO_GROUND_INTAKE, new SequentialCommandGroup(
      new ParallelCommandGroup(
        drivetrain.transitionCommand(CommandSwerveDrivetrain.State.AUTO_INTAKE),
        intake.transitionCommand(Intake.State.INTAKING),
        indexer.transitionCommand(Indexer.State.INDEXING)),
      indexer.waitForState(Indexer.State.HAS_NOTE),
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
      ),
      new WaitUntilCommand(() -> indexer.getState() == Indexer.State.IDLE),
      transitionCommand(State.LOST_NOTE)                                                                                              
    ));

    registerStateCommand(State.CLEANSE, 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING),
          intake.transitionCommand(Intake.State.IDLE),
          indexer.transitionCommand(Indexer.State.PASS_THROUGH)
        ),
        new WaitCommand(4),
        new ConditionalCommand(
          transitionCommand(State.TRAVERSING), 
          transitionCommand(State.STUCK_NOTE), 
          () -> indexer.getState() == Indexer.State.IDLE)
      )
    );

    registerStateCommand(State.BASE_SHOT_SPEAKER, new ParallelCommandGroup(
      drivetrain.transitionCommand(CommandSwerveDrivetrain.State.IDLE),     // CHANGE TO X-SHAPE
      shooter.transitionCommand(Shooter.State.BASE_SHOT_SPEAKER)
    ));
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVERSING);
  }

  @Override
  protected void update() {
    
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    TRAVERSING,
    AUTO_GROUND_INTAKE,
    GROUND_INTAKE,
    GROUND_EJECT,
    LOST_NOTE,
    STUCK_NOTE,
    CLEANSE,
    BASE_SHOT_SPEAKER,
    AMP,
    LOB_STRAIGHT,
    LOB_ARC,
    SPEAKER_AA,
    LOB_AA
  }
}
