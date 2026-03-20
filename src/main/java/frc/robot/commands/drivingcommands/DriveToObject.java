package frc.robot.commands.drivingcommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * DriveToObject command for the robot. Uses limelight object detection to drive toward a detected
 * object while centering it in the camera view.
 *
 * @since 2026-01-15
 */
public class DriveToObject extends Command {

  private static final Logger logger = LogManager.getLogger(DriveToObject.class);

  private final Swerve swerve;
  private final PIDController headingController = Constants.DriveConstants.kHeadingController;

  /**
   * Constructor which sets swerve and controller class variables.
   *
   * @param swerve the swerve drive
   */
  public DriveToObject(Swerve swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  /**
   * Reads the limelight object detection data, calculates the angular offset, and drives toward the
   * detected object using PID heading control.
   */
  @Override
  public void execute() {

    try {

      // --- limelight data ---

      double[] dataFromPython =
          LimelightHelpers.getPythonScriptData(VisionConstants.kObjectCameraName);

      logger.info("Data from Python: {}", java.util.Arrays.toString(dataFromPython));

      // --- angle offset calculation ---
      // index 0 is x of brightest detected point - convert pixel offset to degrees then radians

      double angleDiff =
          (dataFromPython[0] - VisionConstants.kObjectCameraPixelsHorizontal / 2)
              / VisionConstants.kObjectCameraPixelsHorizontal
              * VisionConstants.kObjectCameraFovHorizontal;

      angleDiff *= (Math.PI);
      angleDiff /= 180;

      logger.info("Angle difference: {}", angleDiff);

      // --- heading pid ---
      // drive the angle error to zero

      double rotationOutput = headingController.calculate(angleDiff, 0);

      // --- drive ---
      // if detection confidence is high enough, drive toward it, otherwise spin to search

      if (dataFromPython[2] > VisionConstants.kObjectDetectionThreshold) {
        swerve.drive(new Translation2d(1, 0), rotationOutput, false);
      } else {
        swerve.drive(new Translation2d(0, 0), 1, false);
      }

    } catch (Exception e) {
      logger.error("Error in DriveToObject command: ", e.getMessage());
    }
  }

  /** {@inheritDoc} This command runs continuously until interrupted. */
  @Override
  public boolean isFinished() {
    return false;
  }
}
