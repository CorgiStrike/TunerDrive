package frc.robot.subsystems.Intake;

import frc.robot.Motors.talonfx.PIDSVGains;
import frc.robot.Motors.talonfx.VelocityTalonFX;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.Constants.Intake.Hardware;
import frc.robot.Sensor.ProxSensor;

public class IntakeIOReal implements IntakeIO{
    protected final VelocityTalonFX topMotor =
        new VelocityTalonFX(Hardware.TOP_ID,"rio",Hardware.TOP_GAINS, Hardware.topRatio);
    protected final VelocityTalonFX bottomMotor =
        new VelocityTalonFX(Hardware.BOTTOM_ID,"rio",Hardware.TOP_GAINS, Hardware.bottomRatio);

    protected final ProxSensor prox = new ProxSensor(Hardware.PROX_ID);

    public IntakeIOReal(){
        this(false);
    }

    public IntakeIOReal(boolean sim){
        if (!sim) configureCurrentLimits();

        configureHardware();
    }

    private void configureCurrentLimits(){
        bottomMotor.getConfigurator().apply(Hardware.CURRENT_LIMIT);
        topMotor.getConfigurator().apply(Hardware.CURRENT_LIMIT);
    }

    private void configureHardware(){
        topMotor.setNeutralMode(Hardware.NEUTRAL_MODE);
        bottomMotor.setNeutralMode(Hardware.NEUTRAL_MODE);

        topMotor.setInverted(Hardware.TOP_INVERTED);
    }

    @Override
    public void setVoltage(double voltage){
        topMotor.setVoltage(voltage);
    }

    @Override
    public void setBeltTargetVelocity(double velocity){
        resetFollower();

        topMotor.setTarget(velocity);
    }

    @Override
    public void resetFollower(){
        bottomMotor.setControl(new Follower(Hardware.TOP_ID, Hardware.BOTTOM_INVERTED));
    }

    @Override
    public void stop(){
        topMotor.setTarget(0);

        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    @Override
    public void setGains(PIDSVGains gains) {
    Slot0Configs configs =
        new Slot0Configs()
            .withKP(gains.getP())
            .withKI(gains.getI())
            .withKD(gains.getD())
            .withKS(gains.getS())
            .withKV(gains.getV());

    topMotor.getConfigurator().apply(configs);
    bottomMotor.getConfigurator().apply(configs);
  }
  
    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.velocity = topMotor.getEncoderVelocity();
        inputs.targetVelocity = topMotor.getTarget();
        inputs.voltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.proxTripped = prox.isActivated();
        inputs.rotorVelocity = topMotor.getRotorVelocity().getValueAsDouble();
    }
}

