package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

/**
 * MovementOriented command for the robot. Orients the robot heading
 * to match the direction of joystick movement.
 * 
 * @since 2026-01-20
 */
public class MovementOriented extends Command {

  private final Swerve swerve;
  private final Controller controller;
  // heading controller
  private final PIDController headingController = Constants.DriveConstants.kHeadingController;

  /**
   * MovementOriented constructor for the robot.
   * 
   * @param subsystem  the swerve subsystem
   * @param controller the controller
   * @since 2026-01-21
   */
  public MovementOriented(Swerve subsystem, Controller controller) {
    swerve = subsystem;
    this.controller = controller;
    addRequirements(swerve);
  }

  /**
   * Initializes the command. Currently a no-op.
   *
   * @todo Remove this unless needed
   */
  public void initialize() {
  }

  /**
   * Executes the MovementOriented command.
   * Periodically updates the translation and heading distance to the target pose
   * by subtracting the current pose from the target pose.
   *
   * @since 2026-01-21
   */
  @Override
  public void execute() {
    // rotation oriented to movement direction
    // get movement direction
    Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
    double movementDirection = Math.atan2(translation.getY(), translation.getX());
    // set the robot's target angle to the movement direction
    headingController.setSetpoint(movementDirection);
    double rotationOutput = headingController.calculate(swerve.getPose().getRotation().getRadians());

    double currentGear = Swerve.getCurrentGear();
    swerve.drive(translation.times(currentGear), rotationOutput, true);
  }
}
