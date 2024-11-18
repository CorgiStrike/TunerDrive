package frc.robot.subsystems.Shooter.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    
    @AutoLog
    public class ArmIOInputs {

        public double motorPosition = 0.0;
        
        public double targetPosition = 0.0;

        public double encoderPosition = 0.0;

        public double motorRotorVelocity = 0.0;

        public double motorVelocity = 0.0;

        public double motorVoltage = 0.0;

        public double followerPosition = 0.0;

        public double followerVelocity = 0.0;

      }

      public default void updateInputs(ArmIOInputs inputs) {}

      public default void setTargetPosition(double position) {}

      public default void stop() {}

      public default void syncToAbsoluteEncoder() {}

      public default void resetFollower() {}
}
