package frc.robot.subsystems.Shooter.Flywheels;

import static frc.robot.Constants.Flywheel.Hardware.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Motors.talonfx.VelocityMotionMagicTalonFX;

public class FlywheelsIOReal implements FlywheelsIO {
    protected final VelocityMotionMagicTalonFX topMotor =
        new VelocityMotionMagicTalonFX(
            TOP_MOTOR_ID, GAINS, TOP_MOTOR_RATIO, ACCELERATION, JERK);
    protected final VelocityMotionMagicTalonFX bottomMotor =
        new VelocityMotionMagicTalonFX(
            BOTTOM_MOTOR_ID, BOTTOM_MOTOR_GAINS, BOTTOM_MOTOR_RATIO, ACCELERATION, JERK);

    public FlywheelsIOReal() {
        this(false);
    }

    public FlywheelsIOReal(boolean sim) {
        if (!sim) configureCurrentLimits();
        configureHardware();

        topMotor.setFOC(ENABLE_FOC);
        bottomMotor.setFOC(ENABLE_FOC);
    }

    @Override
    public void updateInputs(FlywheelInputs inputs) {
        inputs.targetVelocity = topMotor.getTarget();
        inputs.velocity = topMotor.getEncoderVelocity();
        inputs.voltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.rotorVelocity = topMotor.getRotorVelocity().getValueAsDouble();

        inputs.velocityBottom = bottomMotor.getEncoderVelocity();
        inputs.targetVeloBottom = bottomMotor.getTarget();
    }

    @Override
    public void setFlywheelTarget(double target) {
        topMotor.setTarget(target);
        bottomMotor.setTarget(target);
    }

    @Override
    public void setFlywheelTargets(double targetTop, double targetBottom) {
        topMotor.setTarget(targetTop);
        bottomMotor.setTarget(targetBottom);
    }

    @Override
    public void setVoltage(double voltage) {
        topMotor.setVoltage(voltage);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        topMotor.setControl(new DutyCycleOut(dutyCycle).withEnableFOC(ENABLE_FOC));
        bottomMotor.setControl(new DutyCycleOut(dutyCycle).withEnableFOC(ENABLE_FOC));
    }

    @Override
    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    @Override
    public void resetFollower() {
        // bottomMotor.setControl(new Follower(TOP_MOTOR_ID, BOTTOM_MOTOR_INVERTED));
    }

    private void configureHardware() {
        topMotor.setNeutralMode(NEUTRAL_MODE);
        bottomMotor.setNeutralMode(NEUTRAL_MODE);

        topMotor.setInverted(TOP_MOTOR_INVERTED);
        bottomMotor.setInverted(BOTTOM_MOTOR_INVERTED);
    }

    private void configureCurrentLimits() {
        topMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
        bottomMotor.getConfigurator().apply(CURRENT_LIMITS_CONFIGS);
    }
}
