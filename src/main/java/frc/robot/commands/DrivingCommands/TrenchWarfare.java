package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

/**
 * TrenchWarfare command for the robot. Drives the robot toward the nearest
 * trench Y-coordinate while allowing driver X-axis control.
 * 
 * @since 2026-01-20
 */
public class TrenchWarfare extends Command {
  private final Swerve swerve;
  private double targetY;
  private double targetHeading = 0;
  private final PIDController yController = Constants.DriveConstants.kYController;
  private final Controller controller;


  private final PIDController headingController = Constants.DriveConstants.kHeadingController;

  /**
   * TrenchWarfare constructor for the robot.
   * 
   * @param subsystem the swerve subsystem
   * @param controller the controller
   * @since 2026-01-21
   */
  public TrenchWarfare(Swerve subsystem, Controller controller) {
    swerve = subsystem;
    this.controller = controller;
    if (swerve.getPose().getY() > Constants.TrenchConstants.kFieldMidlineY) {
        targetY = Constants.TrenchConstants.kUpperTargetY;
    }
    else {
        targetY = Constants.TrenchConstants.kLowerTargetY;
    }
  }

  /**
   * Executes the TrenchWarfare command.
   * Periodically updates the translation and heading distance to the target pose
   * by subtracting the current pose from the target pose.
   * 
   * @since 2026-01-21
   */
  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    double yCalculated = yController.calculate(currentPose.getY(), targetY);
    double headingCalculated = headingController.calculate(currentPose.getRotation().getRadians(),targetHeading);

    swerve.drive(new Translation2d(controller.getDriveX(), yCalculated), headingCalculated, true);
  }
}
