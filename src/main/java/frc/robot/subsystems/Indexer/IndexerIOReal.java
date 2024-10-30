package frc.robot.subsystems.Indexer;

import frc.robot.Constants.Indexer.Hardware;
import frc.robot.Motors.talonfx.VelocityTalonFX;
import frc.robot.Sensor.ProxSensor;

public class IndexerIOReal implements IndexerIO {
    //define motor
    protected final VelocityTalonFX indexerMotor = 
        new VelocityTalonFX(Hardware.indexerID, Hardware.indexerGains, Hardware.indexerRatio);

    ProxSensor prox1 = new ProxSensor(Hardware.prox1ID);
    ProxSensor prox2 = new ProxSensor(Hardware.prox2ID);
    ProxSensor prox3 = new ProxSensor(Hardware.prox3ID);

    public IndexerIOReal(){
        configureCurrentLimits();

        configureHardware();
    }


    public void configureCurrentLimits(){
        indexerMotor.getConfigurator().apply(Hardware.currentLimit);
    }

    public void configureHardware(){
        indexerMotor.setNeutralMode(Hardware.neutralMode);
        indexerMotor.setInverted(Hardware.indexerInverted);
    }
    @Override
    public void setBeltTargetVelocity(double velocity) {
        indexerMotor.setTarget(velocity);
    }

    @Override
    public void stop() {
        indexerMotor.setTarget(0);

        indexerMotor.stopMotor();
    }

    @Override
    public void updateInputs (IndexerIOInputs inputs) {
        inputs.velocity = indexerMotor.getEncoderVelocity();
        inputs.rotorVelocity = indexerMotor.getEncoderVelocity();
        inputs.prox1Tripped = prox1.isActivated();
        inputs.prox2Tripped = prox2.isActivated();
        inputs.prox3Tripped = prox3.isActivated();
    }
}
