package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import org.apache.logging.log4j.Level;

/**
 * Constants class for the robot. Contains all robot-wide numeric constants organized into inner
 * classes by subsystem.
 *
 * @since 2025-10
 */
public class Constants {

  public static Level kLogLevel = Level.INFO;
  public static int kXboxControllerPort = 0;

  /**
   * Drive constants for the robot, including controller multipliers, gear ratios, deadband, and PID
   * controllers.
   *
   * @since 2025-10
   */
  public static class DriveConstants {

    // --- Controller Multipliers ---
    public static final double kControllerDriveMultiplier = 1.0;
    public static final double kControllerRotationMultiplier = 2.0;
    public static final double kAutoDriveMultiplier = 4.5;
    public static final double kLOW_GEAR_MULTIPLIER = 1.0;
    public static final double kHIGH_GEAR_MULTIPLIER = 3.0;
    public static final double kGearStep = 1.0;

    // --- Speed Limits & Input Thresholds ---
    public static final double kMinimumDistanceToStop = 0.1;
    public static final double kDeadband = 0.1;
    public static final double kMaxSpeedFeetPerSecond = 20;
    public static final double kLeftTriggerThreshold = 0.25;
    public static final double kRightTriggerThreshold = 0.1;

    // --- PID Gains ---
    private static final double kXP = 1.0;
    private static final double kXI = 0.0;
    private static final double kXD = 0.1;

    private static final double kYP = 1.0;
    private static final double kYI = 0.0;
    private static final double kYD = 0.1;

    private static final double kHeadingP = 4.0;
    private static final double kHeadingI = 0.0;
    private static final double kHeadingD = 0.0;

    // --- PID Controllers ---
    public static final PIDController kXController =
        new PIDController(
            Constants.DriveConstants.kXP,
            Constants.DriveConstants.kXI,
            Constants.DriveConstants.kXD);

    public static final PIDController kYController =
        new PIDController(
            Constants.DriveConstants.kYP,
            Constants.DriveConstants.kYI,
            Constants.DriveConstants.kYD);

    public static final PIDController kHeadingController =
        new PIDController(
            Constants.DriveConstants.kHeadingP,
            Constants.DriveConstants.kHeadingI,
            Constants.DriveConstants.kHeadingD);

    static {
      kHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }
  }

  /**
   * Vision constants for the robot. Make sure to set the camera names in case you have different
   * cameras.
   *
   * @since 2025-10
   */
  public static class VisionConstants {

    // --- Camera Names ---
    public static String kAprilCamera1Name = "ShooterCamera";
    public static String kAprilCamera2Name = "NonShooterCamera";
    public static String kObjectCameraName = "limelight-obj";

    // --- Object Detection Camera Properties ---
    public static Double kObjectCameraFovHorizontal = 70.0;
    public static Double kObjectCameraPixelsHorizontal = 320.0;
    public static Double kObjectDetectionThreshold = 40.0;

    // --- AprilTag Detection Thresholds ---
    public static final double kSingleTagDistanceThreshold = 4;
    public static final double kStdDevScalingFactor = 30;
    public static AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // --- Camera Poses on Robot (robot-relative) ---
    public static Transform3d kShooterCamPosition =
        new Transform3d(
            new Translation3d(-0.2894, -0.3429, 0.19685), new Rotation3d(0, -0.61086524, -1.5707));
    public static Transform3d kIntakeCamPosition =
        new Transform3d(
            new Translation3d(0.0454, 0.40195, 0.4953), new Rotation3d(0, -0.10471976, 1.5707));

    // --- Pose Estimation Standard Deviations ---
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  /**
   * Auto constants for the robot, including trajectory tolerances and timeout values.
   *
   * @since 2026-01
   */
  public static class AutoConstants {

    public static AutoFactory kAutoFactory;
    public static final double kDelayTime = 0.08;
    public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);

    public static final Time kTimeout = Units.Seconds.of(5);
    public static final Distance kEpsilonDist = Units.Inches.of(4);
  }

  /**
   * Shooter constants including motor IDs, PID gains, interpolation data, and field coordinate
   * targets.
   *
   * @since 2025-10
   */
  public static class ShooterConstants {

    // --- Motor IDs ---
    public static final int kTopMotorID = 2;
    public static final int kBottomMotorID = 3;
    public static final int kHoodMotorID = 4;

    // --- Flywheel PID & Feed-Forward ---
    public static final double kShooterP = 0.00002;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterLPF = 0.9;
    public static final double kShooterKV = 0.000181;

    // --- Hood PID & Mechanics ---
    public static final double kHoodP = 0.5;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.05;
    public static final double kHoodLpf = 0.9;
    public static final double kHoodClipping = 0.3;
    public static final double kHoodZeroingOutput = 0.2;
    public static final double kHoodStallCurrentAmps = 45;
    public static final double kHoodMaxAngleRad = 1.4;
    public static final double kHoodMaxRotation = 1.42857; // Hood gear ratio is 17.22 to 1

    // --- Shooter Operating Parameters ---
    public static final double kNominalBatteryVoltage = 12.1;
    public static final double kShooterSpeedToleranceRPM = 25;
    public static final double kShooterIdleSpeed = 2700;
    public static final double kAimRotationTolerance = 0.32; // In radians, tune this value
    public static final double kRobotToShooterOffsetMeters = 0.254;
    public static final double kInterpolationLambda = 100;

    // --- Field Geometry ---
    public static final double kFieldMidlineY = 4.035;
    public static final double kRedAllianceXBoundary = 11.8;
    public static final double kBlueAllianceXBoundary = 4.6;
    public static final double kRedPassingTargetX = 14.4;
    public static final double kBluePassingTargetX = 2.3;
    public static final Translation2d kBlueHubTranslation = new Translation2d(4.62, kFieldMidlineY);
    public static final Translation2d kRedHubTranslation = new Translation2d(11.92, kFieldMidlineY);

    // --- Interpolation Tables (indexed by distance to hub) ---
    public static final double[] kDistancesToHub = {1.6501, 2.1330, 3.000, 3.97, 5.67}; // In meters
    public static final double[] kShooterSpeeds = {
      2900, 3150, 3210, 3500, 4253
    }; // In rotations per minute
    public static final double[] kHoodAngles = {0, 0.13, 0.49, 0.63, 1.02}; // In radians
  }

  /**
   * Intake constants including motor IDs, PID gains, actuation setpoints, and jiggle mode
   * parameters.
   *
   * @since 2025-10
   */
  public static class IntakeConstants {
    public static final int kDriveMotorID = 7;
    public static final int kActuationMotorID = 6;

    /** PID gains for intake actuation when moving in (retracted). */
    public static final double kInP = 0.15;

    public static final double kInI = 0.0;
    public static final double kInD = 0.0;

    /** PID gains for intake actuation when moving out (extended). */
    public static final double kOutP = 0.08;

    public static final double kOutI = 0.0;
    public static final double kOutD = 0.0;

    /** Actuation encoder setpoint when intake is out (extended). */
    public static final double kActuationOutSetpoint = 4.6;

    public static final double kInJigglyPercentage = 0.4;
    public static final double kOutJigglyPercentage = 0.8;
    public static final double kJiggleIntervalSeconds = 0.5;

    /** Drive motor output when intake is in (retracted). */
    public static final double kDriveSpeedIn = 0.0;

    /** Drive motor output when intake is out (running). */
    public static final double kDriveSpeedOut = 0.4;

    public static final double kIntakeActuationP = 0.0;
    public static final double kIntakeActuationI = 0.0;
    public static final double kIntakeActuationD = 0.0;
  }

  /**
   * Hopper constants including motor IDs and default speeds for the spindexer and bob motors.
   *
   * @since 2025-10
   */
  public static class HopperConstants {
    public static final int kSpindexerMotorID = 1;
    public static final int kBobMotorID = 5;
    public static final double kSpindexerSpeed = -0.55;
    public static final double kBobSpeed = 0.6;
  }

  /**
   * Trench warfare constants including field Y-coordinate targets for trench driving mode.
   *
   * @since 2026-01
   */
  public static class TrenchConstants {
    public static final double kFieldMidlineY = 4.03;
    public static final double kUpperTargetY = 7.42;
    public static final double kLowerTargetY = 0.65;
  }

  public static final double kMetersToInches = 39.37;
}
