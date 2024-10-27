package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class OdometryBoundingBox {
  Translation2d bottomLeft;
  Translation2d topRight;

  public OdometryBoundingBox(Translation2d point1, Translation2d point2) {
    this.bottomLeft =
        new Translation2d(
            Math.min(point1.getX(), point2.getX()), Math.min(point1.getY(), point2.getY()));
    this.topRight =
        new Translation2d(
            Math.max(point1.getX(), point2.getX()), Math.max(point1.getY(), point2.getY()));
  }

  public Pose2d correctPose(Pose2d currentPose) {
    Translation2d currentTranslation = currentPose.getTranslation();

    double newX = currentTranslation.getX();
    double newY = currentTranslation.getY();

    // Check if out of bounds
    newX = Math.min(Math.max(newX, bottomLeft.getX()), topRight.getX());
    newY = Math.min(Math.max(newY, bottomLeft.getY()), topRight.getY());

    return new Pose2d(newX, newY, currentPose.getRotation());
  }
}