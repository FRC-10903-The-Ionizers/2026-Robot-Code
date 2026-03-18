package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class TrenchWarfare extends Command {
  /**
   * GoToPoint command for the robot.
   * 
   * @since 2026-01-20
   */
  private final Swerve swerve;
  private double targetY;
  private double targetHeading = 0;
  private final PIDController yController = Constants.DriveConstants.kYController;
  private final Controller controller;

  private final PIDController headingController = Constants.DriveConstants.kHeadingController;

  public TrenchWarfare(Swerve subsystem, Controller controller) {
    /**
     * GoToPoint constructor for the robot.
     * 
     * @args Swerve subsystem, Pose2d targetPose
     * @since 2026-01-21
     * @return void
     */
    swerve = subsystem;
    this.controller = controller;

  }

  public void execute() {
    /**
     * Executes the GoToPoint command.
     * Periodically updates the translation and heading distance to the target pose
     * by subtracting the current pose from the target pose.
     * 
     * @args None
     * @since 2026-01-21
     * @return void
     */
    if (swerve.getPose().getY() > 4.03) {
      targetY = 7.42;
    } else {
      targetY = 0.65;
    }

    Pose2d currentPose = swerve.getPose();
    double yCalculated = yController.calculate(currentPose.getY(), targetY);
    double headingCalculated = headingController.calculate(currentPose.getRotation().getRadians(), targetHeading);

    swerve.drive(new Translation2d(controller.getDriveX(), yCalculated), headingCalculated, true);
  }
}