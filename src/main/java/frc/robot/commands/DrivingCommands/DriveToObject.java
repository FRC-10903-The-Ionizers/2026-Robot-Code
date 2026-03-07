package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import frc.robot.Constants.Vision;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class DriveToObject extends Command {
    /**
    * 
     * @author Max Clemetson
     * @since 2026-01-15
     */

    private static final Logger logger = LogManager.getLogger(DriveToObject.class);

    private final Swerve swerve;
    private final PIDController headingController = Constants.DriveConstants.kHeadingController;

    public DriveToObject(Swerve swerve) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve:     the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */
        try {
            // see available cameras

            double[] dataFromPython = LimelightHelpers.getPythonScriptData("limelight-obj");
            
            logger.info("Data from Python: {}", java.util.Arrays.toString(dataFromPython));
            // index 0 is x value of brightest point, we want to center that
            double angleDiff = (dataFromPython[0] - VisionConstants.kObjectCameraPixelsHorizontal / 2)
                    / VisionConstants.kObjectCameraPixelsHorizontal * VisionConstants.kObjectCameraFovHorizontal;

            angleDiff *= (Math.PI);
            angleDiff /= 180;

            logger.info("Angle difference: {}", angleDiff);
            // feed into pid controller to get rotation output
            double rotationOutput = headingController.calculate(angleDiff, 0);

            if (dataFromPython[2] > VisionConstants.kObjectDetectionThreshold) {
                swerve.drive(new Translation2d(1, 0), rotationOutput, false);
            } else {
                swerve.drive(new Translation2d(0, 0), 1, false);
            }
        } catch (Exception e) {
            logger.error("Error in DriveToObject command: ", e.getMessage());
        }

    }

    public boolean isFinished() {
        return false; // Runs continuously
    }
}