package frc.robot.commands.drivingcommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

/**
 * MovementOriented command for the robot. Orients the robot heading to match
 * the direction of
 * joystick movement.
 *
 * @since 2026-01-20
 */
public class MovementOriented extends Command {

  private final Swerve swerve;
  private final Controller controller;

  // heading pid - keeps robot pointed toward movement direction
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
   * Executes the MovementOriented command. Periodically updates the translation
   * and heading
   * distance to the target pose by subtracting the current pose from the target
   * pose.
   *
   * @since 2026-01-21
   */
  @Override
  public void execute() {

    // --- translation ---

    Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());

    // --- heading ---
    // point the robot toward wherever the joystick is pointing

    double movementDirection = Math.atan2(translation.getY(), translation.getX());

    headingController.setSetpoint(movementDirection);

    double rotationOutput = headingController.calculate(swerve.getPose().getRotation().getRadians());

    // --- drive ---
    // scale translation by current gear before sending to swerve

    double currentGear = Swerve.getCurrentGear();

    swerve.drive(translation.times(currentGear), rotationOutput, true);
  }
}
