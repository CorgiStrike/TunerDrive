package frc.robot.subsystems.Intake;

import static frc.robot.Constants.Intake.Hardware;

import com.ctre.phoenix6.controls.Follower;

import frc.robot.Motors.talonfx.VelocityTalonFX;
import frc.robot.Sensor.ProxSensor;

public class IntakeIOReal implements IntakeIO {
    protected final VelocityTalonFX topMotor = 
        new VelocityTalonFX(Hardware.TOP_ID,Hardware.TOP_GAINS, Hardware.topRatio);
    protected final VelocityTalonFX bottomMotor = 
        new VelocityTalonFX(Hardware.BOTTOM_ID,Hardware.TOP_GAINS, Hardware.bottomRatio);

    ProxSensor prox = new ProxSensor(Hardware.PROX_ID);

    public IntakeIOReal(){
        configureCurrentLimits();

        configureHardware();
    }

    public void configureCurrentLimits(){
        topMotor.getConfigurator().apply(Hardware.CURRENT_LIMIT);
        bottomMotor.getConfigurator().apply(Hardware.CURRENT_LIMIT);
    }

    private void configureHardware(){
        topMotor.setNeutralMode(Hardware.NEUTRAL_MODE);
        bottomMotor.setNeutralMode(Hardware.NEUTRAL_MODE);

        topMotor.setInverted(Hardware.TOP_INVERTED);
    }

    @Override
    public void resetFollower() {
        bottomMotor.setControl(new Follower(Hardware.TOP_ID, Hardware.BOTTOM_INVERTED));
    }

    @Override
    public void setBeltTargetVelocity(double velocity) {
        resetFollower();

        topMotor.setTarget(velocity);
    }

    @Override
    public void stop() {
        topMotor.setTarget(0);

        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.velocity = topMotor.getEncoderVelocity();
        inputs.rotorVelocity = topMotor.getRotorVelocity().getValueAsDouble();
        inputs.targetVelocity = topMotor.getTarget();
        inputs.voltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.proxTripped = prox.isActivated();
    }
}