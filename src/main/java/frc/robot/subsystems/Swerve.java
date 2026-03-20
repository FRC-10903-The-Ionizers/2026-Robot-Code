package frc.robot.subsystems;

import java.io.File;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Swerve subsystem for the robot.
 * 
 * @since 2025-10
 */
public class Swerve extends SubsystemBase {

    private final static Logger logger = LogManager.getLogger(Swerve.class);
    private SwerveDrive swerveDrive;
    private boolean initialized = false;
    private Field2d field = new Field2d();

    private final PIDController xController = Constants.DriveConstants.kXController;
    private final PIDController yController = Constants.DriveConstants.kYController;
    private final PIDController headingController = Constants.DriveConstants.kHeadingController;

    private static boolean isLockedPosition = false;
    public static double currentGear = 1.0;

    /**
     * Swerve constructor for the robot.
     * 
     * @since 2025-10
     */
    public Swerve() {
        try {
            double maximumSpeed = Units.feetToMeters(Constants.DriveConstants.kMaxSpeedFeetPerSecond);

            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            // On the RIO, deploy files live here:
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);

            swerveDrive.setCosineCompensator(true);
            swerveDrive.setHeadingCorrection(true);

            swerveDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.kZero));

            logger.debug("Swerve initialized!");
            initialized = true;

        } catch (Exception e) {
            logger.fatal("FAILED TO LOAD SWERVE CONFIG", e);
        }
    }

    /**
     * Periodic method for the swerve subsystem.
     * Updates the field position of the robot.
     * 
     * @since 2025-10
     */
    public void periodic() {
        if (!initialized)
            return;

        field.setRobotPose(swerveDrive.getPose());
        SmartDashboard.putData(this);
    }

    /**
     * Resets the odometry of the robot.
     * 
     * @param pose the pose to reset odometry to
     * @return the new pose after resetting
     * @since 2025-10
     */
    public Pose2d resetOdometry(Pose2d pose) {
        if (!initialized)
            return new Pose2d();
        swerveDrive.resetOdometry(pose);
        return swerveDrive.getPose();
    }

    /**
     * Gets the field position of the robot.
     * 
     * @return the Field2d object
     * @since 2025-10
     */
    public Field2d getField() {
        return field;
    }

    /**
     * Gets the pose of the robot.
     * 
     * @return the current Pose2d
     * @since 2025-10
     */
    public Pose2d getPose() {
        if (!initialized)
            return new Pose2d();

        return swerveDrive.getPose();
    }

    /**
     * Follows a trajectory for the robot.
     * 
     * @param sample the swerve sample to follow
     * @since 2025-10
     */
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = swerveDrive.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rotation = speeds.omegaRadiansPerSecond;

        swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotation, true, false);
    }

    /**
     * Gets the swerve drive object.
     * 
     * @return the SwerveDrive instance
     * @since 2025-10
     */
    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /**
     * Adds a vision measurement to the swerve drive, uses standard
     * deviations to weight the measurement along with swerve drive's odometry.
     * 
     * @param visionPose        the vision-estimated pose
     * @param timestamp         the timestamp of the vision measurement
     * @param estimationStdDevs the standard deviations for the measurement
     * @since 2025-10
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        if (!initialized) {
            return;
        }

        swerveDrive.addVisionMeasurement(visionPose, timestamp, estimationStdDevs);
    }

    /**
     * Gets the current gear of the robot.
     * 
     * @return the current gear multiplier
     * @since 2026-01
     */
    public static double getCurrentGear() {
        return currentGear;
    }

    /**
     * Sets the current gear of the robot.
     * 
     * @param gearMultiplier the gear multiplier to set
     * @since 2026-01
     */
    public static void setCurrentGear(double gearMultiplier) {
        currentGear = gearMultiplier;
    }

    /**
     * Drives the robot.
     * 
     * @param translation     the translation vector
     * @param rotation        the rotation speed
     * @param isFieldRelative whether driving is field-relative
     * @since 2026-01
     */
    public void drive(Translation2d translation, double rotation, boolean isFieldRelative) {
        if (isLockedPosition) {
            // locked position (x position)
            swerveDrive.lockPose();
            // lock the brakes
            swerveDrive.setMotorIdleMode(true);
            return;
        } else {
            swerveDrive.drive(translation.times(currentGear), rotation, isFieldRelative, false);
        }
    }
}
