package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

/**
 * TeleopDriveCommand command for the robot. Literally just drives the robot.
 * 
 * @since 2026-01-15
 */
public class TeleopDriveCommand extends Command {
    private final Swerve swerve;
    private final Controller controller;

    /**
     * Constructor which sets swerve and controller class variables.
     * 
     * @param swerve     the swerve drive
     * @param controller the controller
     */
    public TeleopDriveCommand(Swerve swerve, Controller controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    /**
     * Drives the swerve drivetrain using joystick inputs from the
     * controller, respecting the current field-relative mode.
     */
    @Override
    public void execute() {
        if (controller.isFieldRelative()) {
            swerve.drive(
                    new Translation2d(controller.getDriveX(), controller.getDriveY()),
                    controller.getRotation(),
                    true);
        } else {
            swerve.drive(
                    new Translation2d(controller.getDriveX(), controller.getDriveY()),
                    controller.getRotation(),
                    controller.isFieldRelative());
        }
    }

    /** {@inheritDoc} This command runs continuously until interrupted. */
    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
}
