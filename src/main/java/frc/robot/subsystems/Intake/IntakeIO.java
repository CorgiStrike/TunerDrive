package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double velocity = 0.0;
        public double rotorVelocity = 0.0;
        public double targetVelocity = 0.0;
        public double voltage = 0.0;

        public boolean proxTripped = false;
    }

    public default void resetFollower() {}

    public default void setBeltTargetVelocity(double velocity) {}

    public default void stop () {}


    public default void updateInputs(IntakeIOInputs inputs) {}
}