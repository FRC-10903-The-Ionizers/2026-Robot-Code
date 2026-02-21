package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class DriveWhileLocked extends Command {
    /**
     * DriveWhileLocked command for the robot. Drives the robot while maintaining a locked orientation.
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Controller controller;
    private final Pose2d targetPose;
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

    public DriveWhileLocked(Swerve swerve, Controller controller, Pose2d targetPose) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve: the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.controller = controller;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */
        Pose2d currentPosition = swerve.getPose();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();
        Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
        double target_angle = Math.atan2(this.targetPose.getY() - currentY, this.targetPose.getX() - currentX);


        headingController.setSetpoint(target_angle);
        double rotation = headingController.calculate(swerve.getPose().getRotation().getRadians());
        swerve.drive(translation, rotation, controller.isFieldRelative());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
}