package frc.robot.subsystems.Shooter.Arm;

import static frc.robot.Constants.Shooter.Arm.Hardware.*;
import static frc.robot.Constants.Shooter.Arm.Settings.*;

import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.Motors.talonfx.MotionMagicTalonFX;


public class ArmIOReal implements ArmIO{
    protected final MotionMagicTalonFX leaderMotor =
        new MotionMagicTalonFX(LEADER_ID, GAINS, MOTOR_RATIO, VELOCITY, ACCELERATION, JERK);
    
    protected final MotionMagicTalonFX followerMotor =
        new MotionMagicTalonFX(FOLLOWER_ID, GAINS, MOTOR_RATIO, VELOCITY, ACCELERATION, JERK);

    private final AnalogPotentiometer potentiometer =
      new AnalogPotentiometer(
          POTENTIOMETER_ID,
          POTENTIOMETER_RATIO * (POTENTIOMETER_INVERTED ? -1 : 1),
          POTENTIOMETER_OFFSET);
    
    public ArmIOReal () {
        configureCurrentLimits();
        configureHardware();
        syncToAbsoluteEncoder();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.encoderPosition = potentiometer.get();
        inputs.motorPosition = leaderMotor.getEncoderPosition();
        inputs.targetPosition = leaderMotor.getTarget();
        inputs.motorVelocity = leaderMotor.getEncoderVelocity();
        inputs.motorVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorRotorVelocity = leaderMotor.getRotorVelocity().getValueAsDouble();

        inputs.followerPosition = followerMotor.getEncoderPosition();
        inputs.followerVelocity = followerMotor.getEncoderVelocity();
    }

    @Override
    public void resetFollower() {
        followerMotor.setControl(new Follower(LEADER_ID, false));
    }

    @Override
    public void setTargetPosition(double position) {
        if (position < MAX_ANGLE && position > MIN_ANGLE) {
            resetFollower();
            leaderMotor.setTarget(position);
        }
    }
    
    @Override
    public void stop() {
      leaderMotor.setTarget(leaderMotor.getEncoderPosition());
      leaderMotor.stopMotor();
      followerMotor.stopMotor();
    }

    @Override
    public void syncToAbsoluteEncoder() {
      leaderMotor.resetPosition(potentiometer.get());
      followerMotor.resetPosition(potentiometer.get());
    }

    private void configureCurrentLimits() {
        leaderMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
        followerMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    }

    private void configureHardware() {
        leaderMotor.setNeutralMode(NEUTRAL_MODE);
        followerMotor.setNeutralMode(NEUTRAL_MODE);
    
        leaderMotor.setInverted(LEADER_INVERTED);
        followerMotor.setInverted(FOLLOWER_INVERTED);
      }

}
