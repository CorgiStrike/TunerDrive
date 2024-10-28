package frc.robot.Motors.talonfx;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

public class VelocityMotionMagicTalonFX extends MotionMagicTalonFX {
  private MotionMagicVelocityVoltage mmReq = new MotionMagicVelocityVoltage(0);

  public VelocityMotionMagicTalonFX(
      int deviceNumber,
      String canbus,
      PIDSVGains gains,
      double inputToOutputRatio,
      double maxAccel,
      double jerk) {
    super(deviceNumber, canbus, gains, inputToOutputRatio, 10000, maxAccel, jerk);
  }

  public VelocityMotionMagicTalonFX(
      int deviceNumber,
      String canbus,
      PIDSVGains gains,
      double inputToOutputRatio,
      double maxAccel) {
    super(deviceNumber, canbus, gains, inputToOutputRatio, 10000, maxAccel);
  }

  public VelocityMotionMagicTalonFX(int deviceNumber, PIDSVGains gains, double inputToOutputRatio) {
    super(deviceNumber, gains, inputToOutputRatio);
  }

  public VelocityMotionMagicTalonFX(
      int deviceNumber, PIDSVGains gains, double inputToOutputRatio, double maxAccel) {
    super(deviceNumber, gains, inputToOutputRatio, 10000, maxAccel);
  }

  public VelocityMotionMagicTalonFX(
      int deviceNumber, PIDSVGains gains, double inputToOutputRatio, double maxAccel, double jerk) {
    super(deviceNumber, gains, inputToOutputRatio, 10000, maxAccel, jerk);
  }

  public VelocityMotionMagicTalonFX(
      int deviceNumber, String canbus, PIDSVGains gains, double inputToOutputRatio) {
    super(deviceNumber, canbus, gains, inputToOutputRatio);
  }

  @Override
  public void setTarget(double target) {
    this.target = target;
    setControl(mmReq.withVelocity(outputToTicks(target)).withSlot(0));
  }
}
