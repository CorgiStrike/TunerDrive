// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;

import frc.robot.Motors.talonfx.PIDSVGains;
import frc.robot.Vision.DefaultPostProcessor;
import frc.robot.Vision.DefaultPreProcessor;
import frc.robot.Vision.Vision.PVCamera;
import frc.robot.util.AllianceManager;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double defaultTImeout = 4.0;

  // Whether to use the old tuning (soft, original notes) or the new tuning (hard, new notes)
  public static final boolean USE_ORIGINAL_TUNING = true;

  public static final CurrentLimitsConfigs DEFAULT_CURRENT_LIMIT =
      new CurrentLimitsConfigs().withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);

public static final class PhysicalConstants {
    // METERS

    public static final Pose3d CHASSIS_TO_SHOOTER =
        new Pose3d(
            new Translation3d(Units.inchesToMeters(-2.999766), 0, Units.inchesToMeters(9.434239)),
            new Rotation3d(Math.toRadians(90), 0, Math.toRadians(180)));

    // how much taller climber is than shooter pivot when stowed
    public static double CLIMBER_Y_DISTANCE_FROM_SHOOTER_PIVOT = 0.0;

    // how far away climber is from shooter pivot on front/back axis
    public static double CLIMBER_X_DISTANCE_FROM_SHOOTER_PIVOT = 0.0;

    public static Pose2d BLUE_CORNER =
        new Pose2d(new Translation2d(0, Units.feetToMeters(27)), new Rotation2d());

    public static Pose2d BLUE_SPEAKER =
        new Pose2d(new Translation2d(-0.039243, 5.557), Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_AMP =
        new Pose2d(new Translation2d(1.799, Units.feetToMeters(27)), Rotation2d.fromDegrees(-90));
    public static Pose2d BLUE_SOURCE =
        new Pose2d(
            new Translation2d(
                Units.feetToMeters(27) + Units.inchesToMeters(289.986596),
                Units.feetToMeters(27.0 / 2) - Units.inchesToMeters(139.668)),
            Rotation2d.fromDegrees(120));
    public static Pose2d BLUE_CENTER_TRAP =
        new Pose2d(new Translation2d(5.286, 4.115), Rotation2d.fromDegrees(0));
    public static Pose2d BLUE_LEFT_TRAP =
        new Pose2d(new Translation2d(4.597, 4.513), Rotation2d.fromDegrees(120));
    public static Pose2d BLUE_RIGHT_TRAP =
        new Pose2d(new Translation2d(4.5969682, 3.717244813), Rotation2d.fromDegrees(-120));

    public static Pose2d BLUE_LOB_CORNER =
        new Pose2d(2, Units.feetToMeters(27) - 1, new Rotation2d(0));

    public static double TRAP_SHOT_DISTANCE = 1; // Meters

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

    static {
      try {
        APRIL_TAG_FIELD_LAYOUT =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (Exception e) {
        throw new RuntimeException("Could not load AprilTag field layout from WPI");
      }
    }
  }
  public static boolean doubleEqual(double a, double b, double accuracy) {
    return Math.abs(a - b) < accuracy;
  }

  public static boolean doubleEqual(double a, double b) {
    return doubleEqual(a, b, 0.00001);
  }

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
    
    public static final double LEFT_SHOOTER_CAM_TRUST_CUTOFF = Units.feetToMeters(18);
    public static final double RIGHT_SHOOTER_CAM_TRUST_CUTOFF = Units.feetToMeters(18);
    public static final double LEFT_INTAKE_CAM_TRUST_CUTOFF = Units.feetToMeters(18);
    public static final double RIGHT_INTAKE_CAM_TRUST_CUTOFF = Units.feetToMeters(18);

    public static final int AMBIGUITY_AVG_LENGTH = 100;
    public static final double DISTANCE_SCALAR = 2;
    
    public static PVCamera LEFT_SHOOTER_CAM =
      new PVCamera(
        "pv_instance_1", 
        LEFT_SHOOTER_CAM_POSE,
        new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
        new DefaultPostProcessor(FIELD_LAYOUT, LEFT_SHOOTER_CAM_TRUST_CUTOFF)
      );

    public static PVCamera RIGHT_SHOOTER_CAM =
      new PVCamera(
        "pv_instance_4", 
        RIGHT_SHOOTER_CAM_POSE,
        new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
        new DefaultPostProcessor(FIELD_LAYOUT, RIGHT_SHOOTER_CAM_TRUST_CUTOFF)
      );
  
    public static PVCamera LEFT_INTAKE_CAM =
      new PVCamera(
        "pv_instance_2", 
        LEFT_INTAKE_CAM_POSE,
        new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
        new DefaultPostProcessor(FIELD_LAYOUT, LEFT_INTAKE_CAM_TRUST_CUTOFF)
      );

    public static PVCamera RIGHT_INTAKE_CAM =
      new PVCamera(
        "pv_instance_3", 
        RIGHT_INTAKE_CAM_POSE,
        new DefaultPreProcessor(AMBIGUITY_THRESHHOLD, DISTANCE_SCALAR, AMBIGUITY_AVG_LENGTH),
        new DefaultPostProcessor(FIELD_LAYOUT, RIGHT_INTAKE_CAM_TRUST_CUTOFF)
      );
  }

  public static final class Intake {
    public static final class Hardware {

    public static final int TOP_ID = 30;
    public static final int BOTTOM_ID = 31;
    public static final int PROX_ID = 0;

    public static final int topRatio = 1;
    public static final int bottomRatio = 1;

    public static final boolean TOP_INVERTED = false;
    public static final boolean BOTTOM_INVERTED = false;

    public static final double BELT_SPEED = 3000 / 60.0; // rps

    public static CurrentLimitsConfigs CURRENT_LIMIT = DEFAULT_CURRENT_LIMIT;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;


    public static final PIDSVGains TOP_GAINS =
          new PIDSVGains(0.5, 0, 0, 0.2469, 0.1237);
    }
  }

  public static final class Indexer {

    public static final class Hardware {

      public static final int indexerID = 50;

      public static final double indexerRatio = 1;

      public static final boolean indexerInverted = false;

      public static final double indexingSpeed = 1000 / 60.0; // rps
      public static final double passThroughSpeed = 2000 / 60.0; // rps
      public static final double feedSpeed = 50.0;
      public static final double humanPlayerIntakeSpeed = -1000 / 60.0; // rps

      public static final double indexerTimeout = defaultTImeout;
      public static final double ejectTimeout = 5.0;
      public static final double shootTimeout = 5.0;

      public static final int prox1ID = 1;
      public static final int prox2ID = 3;
      public static final int prox3ID = 2;

      public static final CurrentLimitsConfigs currentLimit = DEFAULT_CURRENT_LIMIT;

      public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;

      public static final PIDSVGains indexerGains =
              new PIDSVGains(0.25, 0, 0, 0.1454, 0.1204);
    }

  }

  public static final class Shooter {
    public static final class Sim {}

    public static final class Hardware {}

    public static final class Settings {

      public static final InterpolatingDoubleTreeMap FLYWHEEL_SPEAKER_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap ORIGINAL_SPEAKER_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap HARD_SPEAKER_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap SPEAKER_LUT =
          USE_ORIGINAL_TUNING ? ORIGINAL_SPEAKER_LUT : HARD_SPEAKER_LUT;

      public static final InterpolatingDoubleTreeMap FLYWHEEL_TRAP_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();
      public static final InterpolatingDoubleTreeMap ARM_TRAP_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final double SPEAKER_TARGET_HEIGHT = 2.2;
      public static final InterpolatingDoubleTreeMap ORIGINAL_ARM_LOB_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap HARD_ARM_LOB_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap HARD_FLYWHEEL_LOB_DISTANCE_LUT =
          new InterpolatingDoubleTreeMap();

      public static final InterpolatingDoubleTreeMap ARM_LOB_DISTANCE_LUT =
          USE_ORIGINAL_TUNING ? ORIGINAL_ARM_LOB_DISTANCE_LUT : HARD_ARM_LOB_DISTANCE_LUT;
      public static final InterpolatingDoubleTreeMap FLYWHEEL_LOB_DISTANCE_LUT =
          USE_ORIGINAL_TUNING ? ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT : HARD_FLYWHEEL_LOB_DISTANCE_LUT;

      public static final double TRAP_TARGET_HEIGHT = 1.52;

      static {
        // FLYWHEEL SPEAKER VALUES
        FLYWHEEL_SPEAKER_DISTANCE_LUT.put(0.0, 4000 / 60.0);
        FLYWHEEL_SPEAKER_DISTANCE_LUT.put(Units.feetToMeters(5), 4000 / 60.0);
        FLYWHEEL_SPEAKER_DISTANCE_LUT.put(
            Units.feetToMeters(10), Flywheel.Settings.PARTIAL_SPINUP_VELOCITY);
        FLYWHEEL_SPEAKER_DISTANCE_LUT.put(20.0, Flywheel.Settings.PARTIAL_SPINUP_VELOCITY);

        // ARM SPEAKER OFFSETS
        ORIGINAL_SPEAKER_LUT.put(0.0, 0.0);
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(8), Math.toRadians(1));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(12), Math.toRadians(2));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(14), Math.toRadians(3));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(16), Math.toRadians(4.5));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(18), Math.toRadians(5.5));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(20), Math.toRadians(7.75));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(22), Math.toRadians(9));
        ORIGINAL_SPEAKER_LUT.put(Units.feetToMeters(24), Math.toRadians(10));
        ORIGINAL_SPEAKER_LUT.put(100.0, Math.toRadians(10));

        // HARD NOTE TUNING
        HARD_SPEAKER_LUT.put(0.0, 0.0);
        HARD_SPEAKER_LUT.put(Units.feetToMeters(8), Math.toRadians(0.75));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(10), Math.toRadians(0.75));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(12), Math.toRadians(1.25));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(14), Math.toRadians(1.75));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(16), Math.toRadians(3));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(18), Math.toRadians(3.75));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(20), Math.toRadians(4.5));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(22), Math.toRadians(5.25));
        HARD_SPEAKER_LUT.put(Units.feetToMeters(24), Math.toRadians(6));
        HARD_SPEAKER_LUT.put(100.0, Math.toRadians(6));

        // FLYWHEEL LOB VALUES
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(50.0, 3500 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(13.0, 3500 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(11.6, 3250 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(10.5, 3250 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(9.3, 2750 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(9.0, 2750 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(7.5, 2750 / 60.0);
        ORIGINAL_FLYWHEEL_LOB_DISTANCE_LUT.put(0.0, 3250 / 60.0);

        // ARM LOB OFFSETS
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(50.0, 50 * (Math.PI / 180));
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(11.6, 50 * (Math.PI / 180));
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(10.5, 50 * (Math.PI / 180));
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(9.3, 50 * (Math.PI / 180));
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(9.0, 53 * (Math.PI / 180));
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(7.5, 59 * (Math.PI / 180));
        ORIGINAL_ARM_LOB_DISTANCE_LUT.put(0.0, 50 * (Math.PI / 180));

        // FLYWHEEL LOB VALUES
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(50.0, 3500 / 60.0);
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(10.25, 3000 / 60.0);
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(9.5, 3000 / 60.0);
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(8.75, 2600 / 60.0);
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(8.0, 2500 / 60.0);
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(7.25, 2400 / 60.0);
        HARD_FLYWHEEL_LOB_DISTANCE_LUT.put(0.0, 2000 / 60.0);

        // HARD ARM LOB OFFSETS
        HARD_ARM_LOB_DISTANCE_LUT.put(50.0, 50 * (Math.PI / 180));
        HARD_ARM_LOB_DISTANCE_LUT.put(10.25, 50 * (Math.PI / 180));
        HARD_ARM_LOB_DISTANCE_LUT.put(9.5, 50 * (Math.PI / 180));
        HARD_ARM_LOB_DISTANCE_LUT.put(8.75, 48 * (Math.PI / 180));
        HARD_ARM_LOB_DISTANCE_LUT.put(8.0, 45 * (Math.PI / 180));
        HARD_ARM_LOB_DISTANCE_LUT.put(7.25, 45 * (Math.PI / 180));
        HARD_ARM_LOB_DISTANCE_LUT.put(0.0, 45 * (Math.PI / 180));
      }
    }
    public static final class Arm{

      public static final class Hardware {
        public static final int LEADER_ID = 12;
        public static final int FOLLOWER_ID = 13;
        public static final int POTENTIOMETER_ID = 0;

        public static final double POTENTIOMETER_RATIO = (10.0 / 58.0) * 10 * 2 * Math.PI;
        public static final double MOTOR_RATIO =
          (10.0 / 64.0) * (18.0 / 50.0) * (10.0 / 58.0) * 2 * Math.PI;

        public static final boolean POTENTIOMETER_INVERTED = true;
        public static final boolean LEADER_INVERTED = true;
        public static final boolean FOLLOWER_INVERTED = true;

        public static final double POTENTIOMETER_OFFSET = Math.toRadians(336.2) + Math.toRadians(20);

        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = DEFAULT_CURRENT_LIMIT;

        public static final PIDSVGains GAINS =
          new PIDSVGains(3, 0, 0, 0.0869, 0.3299);
      }

      public static final class Settings {
        public static final double VELOCITY = 100; // RAD/s
        public static final double ACCELERATION = 150; // RAD/s/s
        public static final double JERK = 10_000; // RAD/s/s/s
  
        public static final double POSITION_READY_TOLERANCE = 2 * (Math.PI / 180); // RAD
  
        public static final double BASE_SHOT_POSITION = 57 * (Math.PI / 180); // RAD - originally 59
        public static final double AUTO_START_POSITION = 55 * (Math.PI / 180); // RAD
        public static final double AMP_POSITION = 50 * (Math.PI / 180); // RAD
        public static final double FULL_STOW_POSITION = 25 * (Math.PI / 180); // RAD
        public static final double PARTIAL_STOW_POSITION = 40 * (Math.PI / 180); // RAD
        public static final double CHUTE_INTAKE_POSITION = 40 * (Math.PI / 180); // RAD
        public static final double LOB_POSITION_STRAIGHT = 20.1 * (Math.PI / 180);
  
        public static final double LOB_POSITION_ARC = 50 * (Math.PI / 180);
  
        public static double TRAP_POSITION = 58 * (Math.PI / 180); // RAD
  
        public static final double AUTO_SYNC_TOLERANCE = 0.1;
        public static final double AUTO_SYNC_MAX_VELOCITY = 0.1; // RAD/s
  
        public static final boolean ENABLE_AUTO_SYNC = false;
        public static final double MIN_TIME_BETWEEN_SYNC = 2.0;
  
        public static final double VOLTAGE_INCREMENT = 0.125;
  
        public static final double MIN_ANGLE = 20.0 * (Math.PI / 180);
        public static final double MAX_ANGLE = 60.0 * (Math.PI / 180);
      }

    }

    public static final class Flywheel {
      public static final class Hardware {
      public static final int TOP_MOTOR_ID = 11;
      public static final int BOTTOM_MOTOR_ID = 10;

      public static final double TOP_MOTOR_RATIO = 1;
      public static final double BOTTOM_MOTOR_RATIO = 1;

      public static final boolean TOP_MOTOR_INVERTED = false;
      public static final boolean BOTTOM_MOTOR_INVERTED = false;

      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

      public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
          new CurrentLimitsConfigs()
              .withSupplyCurrentLimit(40)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyTimeThreshold(1.275);

      public static final PIDSVGains GAINS =
          new PIDSVGains(0.3, 0, 0, 0.3664, 0.115);

      public static final PIDSVGains BOTTOM_MOTOR_GAINS =
              new PIDSVGains(0.3, 0, 0, 0.3664, 0.107);

      public static final double ACCELERATION = 3200;
      public static final double JERK = 500;

      public static final boolean ENABLE_FOC = true;
    }

    public static final class Settings {
      public static final double BASE_SHOT_VELOCITY = 5000 / 60.0; // RPS

      public static final double PARTIAL_SPINUP_VELOCITY = 5520 / 60.0; // RPS

      public static final double SPIN_UP_READY_TOLERANCE = 5; // RPS

      public static final double PASS_THROUGH_SPEED = 500 / 60.0; // RPS

      public static final double CHUTE_INTAKE_SPEED = -1000 / 60.0; // RPS

      public static final double AMP_SPEED_TOP = 140 / 60.0; // RPS
      public static final double AMP_SPEED_BOTTOM = 815 / 60.0; // RPS

      public static double TRAP_SPEED_TOP = 1500 / 60.0; // RPS
      public static double TRAP_SPEED_BOTTOM = 2600 / 60.0; // RPS

      public static final double LOB_SPEED_STRAIGHT_TOP = 5200 / 60.0;
      public static final double LOB_SPEED_STRAIGHT_BOTTOM = 2250 / 60.0;

      public static final double LOB_SPEED_ARC = 3250 / 60.0;

      public static final double VOLTAGE_INCREMENT = 0.25;

      // m/s
      public static final double EXIT_VELOCITY = 19.2;
    }
    }
  }

  public static Pose2d mirror(Pose2d pose) {
    return new Pose2d(
        new Translation2d(
            PhysicalConstants.APRIL_TAG_FIELD_LAYOUT.getFieldLength() - pose.getX(), pose.getY()),
        new Rotation2d(Math.PI).minus(pose.getRotation()));
  }

}
