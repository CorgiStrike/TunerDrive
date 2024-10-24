package frc.robot.Motors.talonfx;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class EnhancedTalonFX extends TalonFX {
  private final double inputToOutputRatio;

  /**
   * Constructor for a TalonFX with gear ratio math included
   *
   * @param deviceNumber CAN ID
   * @param canbus name of the canbus (i.e. for CANivore)
   * @param inputToOutputRatio number to multiply TalonFX rotations by to get output units (i.e.
   *     degree/tick)
   */
  public EnhancedTalonFX(int deviceNumber, String canbus, double inputToOutputRatio) {
    super(deviceNumber, canbus);

    this.inputToOutputRatio = inputToOutputRatio;
  }

  /**
   * Constructor for a TalonFX with gear ratio math included
   *
   * @param deviceNumber CAN ID
   * @param inputToOutputRatio number to multiply TalonFX integrated encoder ticks by to get output
   *     units
   */
  public EnhancedTalonFX(int deviceNumber, double inputToOutputRatio) {
    this(deviceNumber, "", inputToOutputRatio);
  }

  public void configure(NeutralModeValue neutralMode, InvertedValue invertedValue) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    getConfigurator().refresh(config);
    config.NeutralMode = neutralMode;
    config.Inverted = invertedValue;
    getConfigurator().apply(config);
  }

  protected void applyConfiguration(TalonFXConfiguration config) {
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  /**
   * Get the position of the encoder (in output units)
   *
   * @return output units
   */
  public double getEncoderPosition() {
    return ticksToOutput(getRotorPosition().getValue());
  }

  /**
   * Get the velocity of the motor (in output units / second)
   *
   * @return output units / sec
   */
  public double getEncoderVelocity() {
    return ticksToOutput(getRotorVelocity().getValue());
  }

  /**
   * Set a percentage power to the motor
   *
   * @param power decimal from -1 to 1 to set the motor's pwoer to
   */
  public void setManualPower(double power) {
    set(power);
  }

  /**
   * Converts some number of encoder ticks into output units
   *
   * @param ticks encoder ticks
   * @return output units
   */
  public double ticksToOutput(double ticks) {
    return ticks * inputToOutputRatio;
  }

  /**
   * Converts some number of output units into encoder ticks
   *
   * @param output output units
   * @return encoder ticks
   */
  public double outputToTicks(double output) {
    return output / inputToOutputRatio;
  }

  /**
   * @param pos The position (in output units) to which the motor should be reset
   */
  public void resetPosition(double pos) {
    setPosition(outputToTicks(pos));
  }

  /** Resets the motor back to zero */
  public void resetPosition() {
    resetPosition(0);
  }

  /**
   * Configure the motors PIDF loop for motion magic
   *
   * @param gains PIDF gains
   */
  public Slot0Configs configurePIDLoop(Slot0Configs slot0, PIDSVGains gains) {
    // Set the motion magic gains in slot0
    Slot0Configs pidConfigs = slot0;
    pidConfigs.kS = gains.getS();
    pidConfigs.kV = gains.getV();
    pidConfigs.kP = gains.getP();
    pidConfigs.kI = gains.getI();
    pidConfigs.kD = gains.getD();

    return pidConfigs;
  }
}
