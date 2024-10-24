// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import frc.robot.Vision.Vision.PVCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Controller {
    public static final int LEFT_FLIGHT_STICK_ID = 0;
    public static final int RIGHT_FLIGHT_STICK_ID = 1;
    public static final int GAMEPAD_ID = 2;
  }
  
  public static class AutoConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(2, 0.0, 0.0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.6, 0, 0.01);
  }

  public static class Drivetrain {
    
    public static final double MAX_ANGULAR_RATE = 4 * Math.PI;
  }
  public static final class Vision {

    public static Double AMBIGUITY_THRESHHOLD = 0.4;

    public static final AprilTagFieldLayout FIELD_LAYOUT;
    static {
      try {
        FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        throw new RuntimeException("Could not load AprilTag field layout from WPI");
      }
    }

    public static Pose3d LEFT_SHOOTER_CAM_POSE =
        new Pose3d(
            Units.inchesToMeters(12.198133),
            Units.inchesToMeters(12.293625),
            Units.inchesToMeters(8.881022),
            new Rotation3d(Math.toRadians(0), Math.toRadians(-30), Math.toRadians(0)));

    public static Pose3d RIGHT_SHOOTER_CAM_POSE =
        new Pose3d(
            Units.inchesToMeters(11.994638),
            Units.inchesToMeters(-12.276838),
            Units.inchesToMeters(8.712641),
            new Rotation3d(0, Math.toRadians(-60), Math.toRadians(0)));

    public static Pose3d LEFT_INTAKE_CAM_POSE =
        new Pose3d(
            Units.inchesToMeters(-11.832791),
            Units.inchesToMeters(11.802600),
            Units.inchesToMeters(8.920582),
            new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180 - 45)));

    public static Pose3d RIGHT_INTAKE_CAM_POSE =
        new Pose3d(
            Units.inchesToMeters(-11.832791),
            Units.inchesToMeters(-11.802600),
            Units.inchesToMeters(8.920582),
            new Rotation3d(0, Math.toRadians(-30), Math.toRadians(180 + 45)));
    
    public static PVCamera LEFT_SHOOTER_CAM =
        new PVCamera("pv_instance_1", LEFT_SHOOTER_CAM_POSE, AMBIGUITY_THRESHHOLD);
    public static PVCamera RIGHT_SHOOTER_CAM =
        new PVCamera("pv_instance_4", LEFT_SHOOTER_CAM_POSE, AMBIGUITY_THRESHHOLD);
    public static PVCamera LEFT_INTAKE_CAM =
        new PVCamera("pv_instance_2", LEFT_SHOOTER_CAM_POSE, AMBIGUITY_THRESHHOLD);
    public static PVCamera RIGHT_INTAKE_CAM =
        new PVCamera("pv_instance_3", LEFT_SHOOTER_CAM_POSE, AMBIGUITY_THRESHHOLD);
  }
}
