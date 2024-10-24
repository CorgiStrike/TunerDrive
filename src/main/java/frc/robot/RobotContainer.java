// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.SMF.StateMachine;
import frc.robot.controllers.RealControllerBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer extends StateMachine<RobotContainer.State>{
  private RealControllerBindings controllerBindings = new RealControllerBindings();

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final Intake intake = new Intake();

  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

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
    configureBindings();
    registerStateTransitions();
    registerStateCommands();
  }

  private void registerStateTransitions() {
    addTransition(State.TRAVERSING, State.AUTO_GROUND_INTAKE);
    addTransition(State.TRAVERSING, State.GROUND_INTAKE);

    addOmniTransition(State.SOFT_E_STOP);
    addOmniTransition(State.TRAVERSING);
  }

  private void registerStateCommands() {
    registerStateCommand(State.SOFT_E_STOP, new ParallelCommandGroup(
      drivetrain.transitionCommand(CommandSwerveDrivetrain.State.IDLE),
      intake.transitionCommand(Intake.State.IDLE)
    ));
  }

  @Override
  protected void determineSelf() {
    setState(State.SOFT_E_STOP);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public enum State {
    UNDETERMINED,
    SOFT_E_STOP,
    TRAVERSING,
    AUTO_GROUND_INTAKE,
    GROUND_INTAKE
  }
}
