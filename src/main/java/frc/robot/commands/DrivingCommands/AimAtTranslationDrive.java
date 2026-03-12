package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class AimAtTranslationDrive extends Command {
    /**
     * DriveWhileLocked command for the robot. Drives the robot while maintaining a locked orientation.
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Controller controller;
    private final Translation2d targetTranslation;

    private final PIDController headingController = Constants.DriveConstants.kHeadingController;

    public AimAtTranslationDrive(Swerve swerve, Controller controller, Translation2d targetTranslation) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve: the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.controller = controller;
        this.targetTranslation = targetTranslation;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */
        Pose2d currentPose = swerve.getPose();
        double currentX = currentPose.getX();
        double currentY = currentPose.getY();
        Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
        double target_angle = Math.atan2(this.targetTranslation.getY() - currentY, this.targetTranslation.getX() - currentX) + Math.PI/2;

        Translation2d targetHubTranslation = RobotContainer.getTargetHubTranslation();
        double distance = currentPose.getTranslation().getDistance(targetHubTranslation);

        double angleOffset = Math.asin(0.254/distance);
        target_angle = target_angle + angleOffset;
        headingController.setSetpoint(target_angle);
        double rotation = headingController.calculate(swerve.getPose().getRotation().getRadians());
        swerve.drive(translation, rotation, controller.isFieldRelative());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
}