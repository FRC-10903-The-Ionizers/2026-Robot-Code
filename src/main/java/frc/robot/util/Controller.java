package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Controller {
    /**
     * Controller class for the robot.
     * 
     * @author Max Clementson & Jake Xie
     * @since 2026-01-19
     */
    private final XboxController controller;
    private int flippedValue;

    public Controller(int port) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            this.flippedValue = 1;
        } else {
            this.flippedValue = -1;
        }
        controller = new XboxController(port);
    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    }

    // -------------------------------
    // Inner class to manage swerve control
    // -------------------------------
    private boolean fieldRelative = true;

    public void toggleFieldRelative() {
        fieldRelative = !fieldRelative;
    }

    public boolean isFieldRelative() {
        return fieldRelative;
    }

    public boolean getIsAlligningToTarget() {
        return controller.getXButton();
    }

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

    private double applyDeadband(double value) {
        double deadband = 0.1;
        return Math.abs(value) > deadband ? value : 0.0;
    }

}
