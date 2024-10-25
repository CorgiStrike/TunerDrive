package frc.robot.Motors.tuning;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class LinearTuningCommand extends Command {
  private ArrayList<Double> voltage = new ArrayList<>();
  private ArrayList<Double> velocity = new ArrayList<>();

  private final DoubleConsumer setMotorVoltage;
  private final DoubleSupplier getMotorVoltage;
  private final DoubleSupplier getMotorVelocity;

  private final double incrementAmount;

  private int currentIncrement = 0;
  private boolean isFinished = false;

  public LinearTuningCommand(
      Trigger stop,
      Trigger incrementUp,
      Trigger incrementDown,
      DoubleConsumer setMotorVoltage,
      DoubleSupplier getMotorVelocity,
      DoubleSupplier getMotorVoltage,
      double incrementAmount) {
    this.incrementAmount = incrementAmount;

    this.setMotorVoltage = setMotorVoltage;
    this.getMotorVelocity = getMotorVelocity;
    this.getMotorVoltage = getMotorVoltage;

    stop.onTrue(new InstantCommand(() -> isFinished = true));
    incrementDown.onTrue(new InstantCommand(this::incrementDown));
    incrementUp.onTrue(new InstantCommand(this::incrementUp));
  }

  private void incrementUp() {
    if (currentIncrement * incrementAmount < 12 && !isFinished) {
      currentIncrement++;
    }
  }

  private void incrementDown() {
    if (currentIncrement * incrementAmount > -12 && !isFinished) {
      currentIncrement--;
    }
  }

  @Override
  public void initialize() {
    isFinished = false;

    voltage.clear();
    velocity.clear();

    currentIncrement = 0;
  }

  @Override
  public void execute() {
    velocity.add(getMotorVelocity.getAsDouble());
    voltage.add(getMotorVoltage.getAsDouble());

    setMotorVoltage.accept(currentIncrement * incrementAmount);
  }

  @Override
  public void end(boolean interrupted) {
    setMotorVoltage.accept(0);

    var regression =
        new PolynomialRegression(
            velocity.stream().mapToDouble(Double::doubleValue).toArray(),
            voltage.stream().mapToDouble(Double::doubleValue).toArray(),
            1);

    double kV = regression.beta(1);
    double kS = regression.beta(0);

    isFinished = true;

    System.out.printf("Calculated kV -> %.4f\nCalculated kS -> %.4f", kV, kS);
  }

  @Override
  public boolean isFinished() {
    // to avoid overusing memory
    return voltage.size() > 30_000 || isFinished;
  }
}
