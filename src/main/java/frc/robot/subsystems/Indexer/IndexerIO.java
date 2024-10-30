package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {
        public double velocity = 0.0;
        public double rotorVelocity = 0.0;
        public double targetVelocity = 0.0;
        public double voltage = 0.0;

        public boolean prox1Tripped = false;
        public boolean prox2Tripped = false;
        public boolean prox3Tripped = false;
    }

    public default void setBeltTargetVelocity(double velocity) {}

    public default void stop() {}

    public default void updateInputs(IndexerIOInputs inputs) {}


}
