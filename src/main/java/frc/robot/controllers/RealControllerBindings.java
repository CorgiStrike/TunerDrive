package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.HID.CommandFlightStick;

public class RealControllerBindings implements ControllerBindings {

  private final CommandFlightStick leftFlightStick =
      new CommandFlightStick(Constants.Controller.LEFT_FLIGHT_STICK_ID);
  private final CommandFlightStick rightFlightStick =
      new CommandFlightStick(Constants.Controller.RIGHT_FLIGHT_STICK_ID);
  private final CommandXboxController gamepad =
      new CommandXboxController(Constants.Controller.GAMEPAD_ID);

  @Override
  public double getDriveXValue() {
    return rightFlightStick.getY();
  }

  @Override
  public double getDriveYValue() {
    return rightFlightStick.getX();
  }

  @Override
  public double getDriveTurnValue() {
    return -leftFlightStick.getRawAxis(0);
  }

  @Override
  public Trigger resetVisionPose() {
    return rightFlightStick.topRight();
  }

  @Override
  public Trigger resetGyro() {
    return rightFlightStick.topLeft();
  }

  @Override
  public Trigger autoIntake() {
    return rightFlightStick.trigger();
  }

  @Override
  public Trigger manualIntake() {
    return gamepad.a();
  }

  @Override
  public Trigger intakeEject() {
    return gamepad.button(8);
  }

  @Override
  public Trigger cleanseIndexer() {
    return gamepad.button(7);
  }

  @Override
  public Trigger stow(){
    return gamepad.leftBumper();
  }  

  @Override
  public Trigger feedShooter(){
    return gamepad.rightBumper();
  }

  @Override
  public Trigger baseShot(){
    return gamepad.b();
  }
}
