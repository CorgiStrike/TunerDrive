package frc.robot.HID;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class FlightStickImpostor extends FlightStick {

  private final CommandXboxController impostor;
  private final boolean left;

  public FlightStickImpostor(CommandXboxController impostor, int port, boolean left) {
    super(port);

    this.impostor = impostor;
    this.left = left;
  }

  @Override
  public double getXAxis() {
    return left ? impostor.getLeftX() : impostor.getRightX();
  }

  @Override
  public double getYAxis() {
    return left ? impostor.getLeftY() : impostor.getRightY();
  }
}
