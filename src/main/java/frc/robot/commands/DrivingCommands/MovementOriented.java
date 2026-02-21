package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Controller;

public class MovementOriented extends Command {
  /**
   * GoToPoint command for the robot.
   * 
   * @since 2026-01-20
   */

  private final Swerve swerve;
  private final Controller controller;
  // heading controller
  private final PIDController headingController = new PIDController(Constants.DriveConstants.kHeadingP, 0, 0);

  public MovementOriented(Swerve subsystem, Controller controller) {
    /**
     * GoToPoint constructor for the robot.
     * 
     * @args Swerve subsystem, Pose2d targetPose
     * @since 2026-01-21
     * @return void
     */
    swerve = subsystem;
    this.controller = controller;
    addRequirements(swerve);
  }

  public void initialize() {
    /**
     * @todo Remove this unless needed
     */
  }
  
  public void execute() {
    /**
     * Executes the MovementOriented command.
     * Periodically updates the translation and heading distance to the target pose by subtracting the current pose from the target pose.
     * @args None
     * @since 2026-01-21
     * @return void
     */

      //rotation oriented to movement direction
      // get movement direction
      Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
      double movementDirection = Math.atan2(translation.getY(), translation.getX());
      // set the robot's target angle to the movement direction
      headingController.setSetpoint(movementDirection);
      double rotationOutput = headingController.calculate(swerve.getPose().getRotation().getRadians());
      
      double currentGear = swerve.getCurrentGear();
      swerve.drive(translation.times(currentGear), rotationOutput, true);
  };
}
