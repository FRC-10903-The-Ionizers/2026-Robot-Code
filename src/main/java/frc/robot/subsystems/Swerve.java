package frc.robot.subsystems;

import java.io.File;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    /**
     * Swerve subsystem for the robot.
     * 
     * @since 2025-10
     */
    private SwerveDrive swerveDrive;
    private boolean initialized = false;
    private Field2d field = new Field2d();

    private final PIDController xController = new PIDController(
        Constants.DriveConstants.kXP, 
        Constants.DriveConstants.kXI, 
        Constants.DriveConstants.kXD
    );
    private final PIDController yController = new PIDController(
        Constants.DriveConstants.kYP, 
        Constants.DriveConstants.kYI, 
        Constants.DriveConstants.kYD
    );
    private final PIDController headingController = new PIDController(
        Constants.DriveConstants.kHeadingP, 
        Constants.DriveConstants.kHeadingI, 
        Constants.DriveConstants.kHeadingD
    );

    private static boolean isLockedPosition = false;
    public static double currentGear = 1.0;
    private static boolean isMovementOriented = false;

    public Swerve() {
        /**
         * Swerve constructor for the robot.
         * 
         * @args None
         * @since 2025-10
         * @return void
         */
        try {
            headingController.enableContinuousInput(-Math.PI, Math.PI);
            double maximumSpeed = Units.feetToMeters(4.5);

            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            // On the RIO, deploy files live here:
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);

            swerveDrive.setCosineCompensator(true);
            swerveDrive.setHeadingCorrection(true);

            swerveDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.kZero));

            System.out.println("Swerve is initialized!");
            initialized = true;
        } catch (Exception e) {
            System.out.println("Failed to load swerve config.");
            DriverStation.reportError("Failed to load swerve configuration from /deploy/swerve: " + e.getMessage(),
                    e.getStackTrace());
        }
    }

    public void periodic() {
        /**
         * Periodic method for the swerve subsystem.
         * 
         * Updates the field position of the robot.
         * @args None
         * @since 2025-10
         * @return void
         */
        if (!initialized)
            return;

        field.setRobotPose(swerveDrive.getPose());
        SmartDashboard.putData(this);
    }

    public Pose2d resetOdometry(Pose2d pose) {
        /**
         * Resets the odometry of the robot.
         * 
         * @args Pose2d pose
         * @since 2025-10
         * @return Pose2d
         */
        if (!initialized)
            return new Pose2d();
        swerveDrive.resetOdometry(pose);
        return swerveDrive.getPose();
    }

    // Elastic Stuff

    public Field2d getField() {
        /**
         * Gets the field position of the robot.
         * 
         * @args None
         * @since 2025-10
         * @return Field2d
         */
        return field;
    }

    public Pose2d getPose() {
        /**
         * Gets the pose of the robot.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return Pose2d
         */
        if (!initialized)
            return new Pose2d();
            
        return swerveDrive.getPose();
    }

    public void followTrajectory(SwerveSample sample) {
        /**
         * Follows a trajectory for the robot using elastic.
         * 
         * @args SwerveSample sample
         * @since 2025-10
         * @return void
         */

        // Get the current pose of the robot
        Pose2d pose = swerveDrive.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
            );

        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rotation = speeds.omegaRadiansPerSecond;

        swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotation, true, false);  
    }

    public SwerveDrive getSwerveDrive() {
        /**
         * Gets the swerve drive object.
         * 
         * @args None
         * @since 2025-10
         * @return SwerveDrive
         */
        return swerveDrive;
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        /**
         * Adds a vision measurement to the swerve drive, uses standard 
         * deviations to weight the measurement along with swerve drive's odometry.
         * 
         * @args Pose2d visionPose, double timestamp, Matrix<N3, N1> estimationStdDevs
         * @since 2025-10
         * @return void
         */
        if (!initialized){
            return;
        }
        
        swerveDrive.addVisionMeasurement(visionPose, timestamp, estimationStdDevs);
    }

    public static double getCurrentGear() {
        /**
         * Gets the current gear of the robot.
         * 
         * @args None
         * @since 2026-01
         * @return double currentGear
         */
        return currentGear;
    }

    public static void setCurrentGear(double gearMultiplier) {
        /**
         * Sets the current gear of the robot.
         * 
         * @args double gearMultiplier
         * @since 2026-01
         * @return void
         */
        currentGear = gearMultiplier;
    }

    public void drive(Translation2d translation, double rotation, boolean isFieldRelative){
        /**
         * Drives the robot.
         * 
         * @args Translation2d translation, double rotation, boolean isFieldRelative
         * @since 2026-01
         * @return void
         */
        if (isLockedPosition) {
            // locked position (x position)
            swerveDrive.lockPose();
            // lock the brakes
            swerveDrive.setMotorIdleMode(true);
            return;
        }
       else {
            swerveDrive.drive(translation.times(currentGear), rotation, isFieldRelative, false);    
        }
    }
}