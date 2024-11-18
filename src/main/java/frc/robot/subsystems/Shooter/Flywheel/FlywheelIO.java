package frc.robot.subsystems.Shooter.Flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public double velocity = 0.0;
    public double rotorVelocity = 0.0;
    public double targetVelocity = 0.0;
    public double voltage = 0.0;

    public double targetVeloBottom = 0.0;
    public double velocityBottom = 0.0;
  }

  public default void setFlywheelTarget(double target) {}

  public default void setFlywheelTargets(double targetTop, double targetBottom) {}

  public default void stop() {}

  public default void setDutyCycle(double dutyCycle) {}

  public default void resetFollower() {}

  public default void updateInputs(FlywheelIOInputs inputs) {}
}
