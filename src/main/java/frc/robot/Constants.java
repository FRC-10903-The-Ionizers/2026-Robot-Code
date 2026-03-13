package frc.robot;

import java.util.Optional;

import org.apache.logging.log4j.Level;

import choreo.auto.AutoFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    /**
     * Constants class for the robot.
     * 
     * @author Max Clemetson, Jake Xie, Siddhartha Hiremath, Justin Baratta
     * @since 2025-10
     */

    public static Level kLogLevel = Level.DEBUG;
    public static int kXboxControllerPort = 0;

    public static class DriveConstants {
        /**
         * Drive constants for the robot.
         * 
         * @author Max Clemetson, Jake Xie, Siddhartha Hiremath, Justin Baratta
         * @since 2025-10
         */
        public static final double kControllerDriveMultiplier = 1.0;
        public static final double kControllerRotationMultiplier = 2.0;
        public static final double kAutoDriveMultiplier = 4.5;
        public static final double kLOW_GEAR_MULTIPLIER = 1.0;
        public static final double kHIGH_GEAR_MULTIPLIER = 3.0;

        public static final double kMinimumDistanceToStop = 0.1;

        private static final double kXP = 1.0;
        private static final double kXI = 0.0;
        private static final double kXD = 0.1;

        private static final double kYP = 1.0;
        private static final double kYI = 0.0;
        private static final double kYD = 0.1;

        private static final double kHeadingP = 4.0;
        private static final double kHeadingI = 0.0;
        private static final double kHeadingD = 0.0;

        public static final PIDController kXController = new PIDController(
                Constants.DriveConstants.kXP,
                Constants.DriveConstants.kXI,
                Constants.DriveConstants.kXD);

        public static final PIDController kYController = new PIDController(
                Constants.DriveConstants.kYP,
                Constants.DriveConstants.kYI,
                Constants.DriveConstants.kYD);

        public static final PIDController kHeadingController = new PIDController(
                Constants.DriveConstants.kHeadingP,
                Constants.DriveConstants.kHeadingI,
                Constants.DriveConstants.kHeadingD);
        // pid controllers here

        static {
            kHeadingController.enableContinuousInput(-Math.PI, Math.PI);
        }

    }

    public static class VisionConstants {
        /**
         * Vision constants for the robot. Make sure to set the camera names in case u
         * have different cameras.
         * 
         * @author Max Clemetson, Jake Xie, SIddhartha Hiremath
         * @since 2025-10
         */
        public static String kAprilCamera1Name = "ShooterCamera";
        public static String kAprilCamera2Name = "NonShooterCamera";

        public static Double kObjectCameraFovHorizontal = 70.0;
        public static Double kObjectCameraPixelsHorizontal = 320.0;

        public static Transform3d kShooterCamPosition = new Transform3d(new Translation3d(-0.2894, -0.3429, 0.19685), new Rotation3d(0,-0.61086524, -1.5707));
        public static Transform3d kIntakeCamPosition = new Transform3d(new Translation3d(0.0454, 0.40195, 0.4953), new Rotation3d(0, -0.10471976, 1.5707));

        public static Double kObjectDetectionThreshold = 40.0;
        public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);;

        
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class AutoConstants {
        /**
         * Auto constants for the robot.
         * 
         * @author Max Clementson, Jake Xie, Siddhartha Hiremath, Justin Baratta
         * @since 2026-01
         */
        public static AutoFactory kAutoFactory;
        public static final double kDelayTime = 0.08;
        public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);

        public static final Time kTimeout = Units.Seconds.of(5);
        public static final Distance kEpsilonDist = Units.Inches.of(4);
    }

    public static class ShooterConstants {
        public static final int kTopMotorID = 2;
        public static final int kBottomMotorID = 3;
        public static final int kHoodMotorID = 4;
        public static final double kShooterP = 0.01;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.005;
        public static final double kShooterFF = 0.998;
        public static final double kInterpolationLambda = 100;

        public static final Translation2d kBlueHubTranslation = new Translation2d(4.62, 4.035);
        public static final Translation2d kRedHubTranslation = new Translation2d(11.92, 4.035);
        public static final double[] kDistancesToHub = {1.6501, 2.1330, 3.000, 3.97, 5.67}; // In meters
        public static final double[] kShooterSpeeds = {3400, 3600, 4000, 4293, 5003}; // In rotations per minute
        public static final double[] kHoodAngles = {0, 0.2, 0.35, 0.4, 0.67}; // In 
        public static final double kHoodMaxRotation = 1.42857; //Hood gear ration is 17.22 to 1
    }
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

        /** Drive motor output when intake is in (retracted). */
        public static final double kDriveSpeedIn = 0.0;
        /** Drive motor output when intake is out (running). */
        public static final double kDriveSpeedOut = 0.4;

        public static final double kIntakeActuationP = 0.0;
        public static final double kIntakeActuationI = 0.0;
        public static final double kIntakeActuationD = 0.0;
    }

    public static class HopperConstants {
        public static final int kSpindaxerMotorID = 1;
        public static final int kBobMotorID = 5;
    }
}
