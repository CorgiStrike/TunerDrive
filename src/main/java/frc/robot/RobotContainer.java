// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.SMF.StateMachine;
import frc.robot.controllers.RealControllerBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;


public class RobotContainer extends StateMachine<RobotContainer.State>{
  private RealControllerBindings controllerBindings = new RealControllerBindings();

  private final PWM fan = new PWM(9);

  //initialize subsystems
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

  private void configureBindings() {
    drivetrain.configureBindings(controllerBindings::getDriveXValue, controllerBindings::getDriveYValue, controllerBindings::getDriveTurnValue);

    // reset the field-centric heading on left bumper press
    controllerBindings.resetGyro().onTrue(drivetrain.runOnce(() -> drivetrain.swerveDrive.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.swerveDrive.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.swerveDrive.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    super("RobotContainer", State.UNDETERMINED, State.class);

    // Add SMF Children
    addChildSubsystem(drivetrain);

    configureBindings();
    registerStateTransitions();
    registerStateCommands();
  }

  private void registerStateTransitions() {
    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.TRAVERSING);
    addOmniTransition(State.TEST);
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, new ParallelCommandGroup(
      drivetrain.transitionCommand(CommandSwerveDrivetrain.State.IDLE)
    ));

    registerStateCommand(State.TRAVERSING, new ParallelCommandGroup(
      drivetrain.transitionCommand(CommandSwerveDrivetrain.State.TRAVERSING)
    ));

    registerStateCommand(State.TEST, new RunCommand(()->{
      fan.setSpeed(0.5);
    }));
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  @Override
  protected void onTeleopStart() {
    requestTransition(State.TRAVERSING);
  }

  protected void onTestStart() {
    requestTransition(State.TEST);
  }

  @Override
  protected void update() {
    
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    TRAVERSING,
    TEST
  }
}
