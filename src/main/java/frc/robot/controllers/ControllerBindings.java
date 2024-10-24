package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerBindings {

  double getDriveXValue();

  double getDriveYValue();

  double getDriveTurnValue();

  Trigger resetGyro();

  Trigger autoIntake();

  Trigger manualIntake();

  default Trigger resetVisionPose() {
    return new Trigger(() -> false);
  }
}
