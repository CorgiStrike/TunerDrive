package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.HID.CommandFlightStick;

public class RealControllerBindings implements ControllerBindings {

  private final CommandFlightStick leftFlightStick =
      new CommandFlightStick(Constants.Controller.LEFT_FLIGHT_STICK_ID);
  private final CommandFlightStick rightFlightStick =
      new CommandFlightStick(Constants.Controller.RIGHT_FLIGHT_STICK_ID);
  private final XboxController gamepad =
      new XboxController(Constants.Controller.GAMEPAD_ID);
  private final EventLoop loop = CommandScheduler.getInstance().getDefaultButtonLoop();

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
    return leftFlightStick.getRawAxis(0);
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
    return new Trigger(gamepad.a(loop));
  }
}
