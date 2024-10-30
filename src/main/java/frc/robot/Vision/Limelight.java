package frc.robot.Vision;

import frc.robot.LimelightHelpers;

public class Limelight {
    private String name;

    public Limelight(String name) {
        this.name = "limelight";
    }

    public Double getTX() {
        return LimelightHelpers.getTX(name);
    }

    public Double getTY() {
        return LimelightHelpers.getTY(name);
    }

    public Double getTA() {
        return LimelightHelpers.getTA(name);
    }
}
