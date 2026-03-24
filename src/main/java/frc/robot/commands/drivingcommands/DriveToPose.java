package frc.robot.commands.drivingcommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * DriveToPose command for the robot. Drives to a target pose using PID
 * controllers.
 *
 * @since 2026-01-20
 */
public class DriveToPose extends Command {

  private final Swerve swerve;
  private final Pose2d targetPose;

  private final PIDController controllerX = Constants.DriveConstants.kXController;
  private final PIDController controllerY = Constants.DriveConstants.kYController;
  private final PIDController headingController = Constants.DriveConstants.kHeadingController;

  /**
   * DriveToPose constructor for the robot.
   *
   * @param swerve     the swerve subsystem
   * @param targetPose the target pose to drive to
   * @since 2026-01-21
   */
  public DriveToPose(Swerve swerve, Pose2d targetPose) {
    this.swerve = swerve;
    this.targetPose = targetPose;
    addRequirements(swerve);
  }

  /**
   * Executes the DriveToPose command. Periodically updates the translation and
   * heading distance to
   * the target pose by subtracting the current pose from the target pose.
   *
   * @since 2026-01-21
   */
  @Override
  public void execute() {

    Pose2d currentPose = swerve.getPose();

    // --- pid outputs ---
    // each axis gets its own controller, heading separately

    double calcX = controllerX.calculate(currentPose.getX(), targetPose.getX());
    double calcY = controllerY.calculate(currentPose.getY(), targetPose.getY());

    double headingCalculated = headingController.calculate(
        currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    swerve.drive(new Translation2d(calcX, calcY), headingCalculated, true);
  }

  /**
   * Checks if the DriveToPose command is finished based on the minimum distance
   * to stop.
   *
   * @since 2026-01-21
   */
  @Override
  public boolean isFinished() {

    Pose2d currentPose = swerve.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    // done once we're close enough to the target
    if (distance < Constants.DriveConstants.kMinimumDistanceToStop) {
      return true;
    } else {
      return false;
    }
  }
}
