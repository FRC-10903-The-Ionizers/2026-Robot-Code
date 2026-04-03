package frc.robot.commands.drivingcommands;

import java.util.Optional;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class ShootAndDrive extends Command {
    /**
     * DriveWhileLocked command for the robot. Drives the robot while maintaining a
     * locked orientation.
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Controller controller;
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;
    private Timer timer;
    private final double time;
    private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix;

    private final PIDController headingController = Constants.DriveConstants.kHeadingController;

    public ShootAndDrive(Swerve swerve, Shooter shooter, Intake intake, Hopper hopper, Controller controller,
            InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve:     the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.controller = controller;
        this.interpolationMatrix = interpolationMatrix;
        time = 0;
        addRequirements(swerve, hopper, shooter, intake);
    }

    public ShootAndDrive(Swerve swerve, Shooter shooter, Intake intake, Hopper hopper, Controller controller,
            InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix, double time) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve:     the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.hopper = hopper;
        this.controller = controller;
        this.interpolationMatrix = interpolationMatrix;
        timer = new Timer();
        timer.reset();

        this.time = time;
        addRequirements(swerve, hopper, shooter, intake);
    }

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */
        // Get target translation

        if (timer != null && !timer.isRunning()) {
            timer.start();
        }
        Optional<Alliance> ally = DriverStation.getAlliance();

        if (!ally.isPresent()) {
            System.out.println("Alliance not detected!");
            return;
        }

        Translation2d targetTranslation;
        if (ally.get() == Alliance.Red && swerve.getPose().getTranslation().getX() > 11.8) {
            targetTranslation = Constants.ShooterConstants.kRedHubTranslation;
        } else if (ally.get() == Alliance.Blue && swerve.getPose().getTranslation().getX() < 4.6) {
            targetTranslation = Constants.ShooterConstants.kBlueHubTranslation;
        } else {
            double targetPoseY;
            if (this.swerve.getPose().getY() >= 4.035) {
                targetPoseY = (4.035 / 2) * 3;
            } else {
                targetPoseY = 4.035 / 2;
            }

            if (ally.get() == Alliance.Red) {
                targetTranslation = new Translation2d(14.4, targetPoseY);
            } else {
                targetTranslation = new Translation2d(2.3, targetPoseY);
            }
        }

        ChassisSpeeds speeds = swerve.getRobotVelocity();
        double vx = speeds.vxMetersPerSecond;
        double vy = speeds.vyMetersPerSecond;

        Translation2d robotVelocity = new Translation2d(vx * Constants.ShooterConstants.kFlightTime, vy * Constants.ShooterConstants.kFlightTime);
        targetTranslation = targetTranslation.minus(robotVelocity);

        Pose2d currentPose = swerve.getPose();

        SmartDashboard.putNumber("Distance to Hub", (swerve.getPose().getTranslation().getDistance(targetTranslation)));
        double distanceToTarget = currentPose.getTranslation().getDistance(targetTranslation);

        // Drivetrain stuff
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
        double target_angle = Math.atan2(targetTranslation.getY() - currentY, targetTranslation.getX() - currentX)
                + Math.PI / 2;

        double angleOffset = Math.asin(0.254 / distanceToTarget);
        target_angle = target_angle + angleOffset;
        headingController.setSetpoint(target_angle);

        double rotation = headingController.calculate(swerve.getPose().getRotation().getRadians());

        swerve.drive(translation, rotation, controller.isFieldRelative());

        // With distance start getting shooting stuff set
        Matrix<N2, N1> interp = interpolationMatrix.get(distanceToTarget);
        double interpolatedShooterSpeed = 0.0;
        double interpolatedHoodAngle = 0.0;

        if (interp != null) {
            interpolatedShooterSpeed = interp.get(0, 0);
            interpolatedHoodAngle = interp.get(1, 0);
        }

        this.shooter.setSpeedAndAngle(interpolatedShooterSpeed, interpolatedHoodAngle);
        if ((this.shooter.getIsReadyToShoot()) && Math.abs(rotation) < 0.3) {
            hopper.setIsHopperRunning(true);
        } else {
            hopper.setIsHopperRunning(false);
        }

        intake.setJiggly(true);
    }

    @Override
    public boolean isFinished() {
        if (timer == null) {
            return false;
        }
        if (time == 0 || timer.get() < time) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeedAndAngle(0, 0);
        hopper.setIsHopperRunning(false);
        intake.setJiggly(false);

        if (timer != null) {
            timer.stop();
            timer.reset();
        }
    }
}