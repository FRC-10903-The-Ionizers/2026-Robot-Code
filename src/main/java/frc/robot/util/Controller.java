package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

/**
 * Controller class for the robot. Wraps an Xbox controller and provides
 * drive inputs with deadband, field-relative toggle, and alliance-aware
 * axis flipping.
 *
 * @since 2026-01-19
 */
public class Controller {

    private final XboxController controller;
    private int flippedValue;

    /**
     * Constructs the Controller on the specified port. Determines the
     * alliance color to set the axis flipping direction.
     *
     * @param port the USB port of the Xbox controller
     */
    public Controller(int port) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            this.flippedValue = 1;
        } else {
            this.flippedValue = -1;
        }
        controller = new XboxController(port);
    }

    /**
     * Returns the raw left stick X axis value.
     *
     * @return left stick X value in [-1, 1]
     */
    public double getLeftX() {
        return controller.getLeftX();
    }

    /**
     * Returns the raw left stick Y axis value.
     *
     * @return left stick Y value in [-1, 1]
     */
    public double getLeftY() {
        return controller.getLeftY();
    }

    /**
     * Returns the raw right stick X axis value.
     *
     * @return right stick X value in [-1, 1]
     */
    public double getRightX() {
        return controller.getRightX();
    }

    // ---- Swerve control state ----
    private boolean fieldRelative = true;

    /**
     * Toggles field-relative driving on or off.
     */
    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    /**
     * Returns whether the controller is in field-relative mode.
     *
     * @return true if field-relative mode is active
     */
    public boolean isFieldRelative() {
        return fieldRelative;
    }

    /**
     * Returns whether the X button is pressed, indicating the driver
     * wants to align to a target.
     *
     * @return true if the X button is held
     */
    public boolean getIsAlligningToTarget() {
        return controller.getXButton();
    }

    /**
     * Returns whether the A button is pressed, indicating the driver
     * is aiming.
     *
     * @return true if the A button is held
     */
    public boolean getIsAiming() {
        return controller.getAButton();
    }

    /** Get forward/backward, with deadband applied */
    public double getDriveX() {
        return applyDeadband(-controller.getLeftY()) * Constants.DriveConstants.kControllerDriveMultiplier * flippedValue * (isFieldRelative() ? 1 : flippedValue);
    }

    /** Get strafe, with deadband applied */
    public double getDriveY() {
        return applyDeadband(-controller.getLeftX()) * Constants.DriveConstants.kControllerDriveMultiplier * flippedValue * (isFieldRelative() ? 1 : flippedValue);
    }

    /** Get rotation, with deadband applied */
    public double getRotation() {
        return applyDeadband(-controller.getRightX()) * Constants.DriveConstants.kControllerRotationMultiplier;
    }

    /**
     * Applies a deadband to the given value. Returns 0 if the absolute
     * value is within the deadband threshold.
     *
     * @param value the raw axis value
     * @return the value with deadband applied
     */
    private double applyDeadband(double value) {
        return Math.abs(value) > Constants.DriveConstants.kDeadband ? value : 0.0;
    }

}
