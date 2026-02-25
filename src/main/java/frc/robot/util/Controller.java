package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class Controller {
    private final XboxController controller;

    public Controller(int port) {
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
    public class SwerveController {
        private boolean fieldRelative = true;

        public void toggleFieldRelative() {
            fieldRelative = !fieldRelative;
        }

        public boolean isFieldRelative() {
            return fieldRelative;
        }

        /** Get forward/backward, with deadband applied */
        public double getDriveX() {
            return applyDeadband(-controller.getLeftY()) * Constants.DriveConstants.kControllerDriveMultiplier;
        }

        /** Get strafe, with deadband applied */
        public double getDriveY() {
            return applyDeadband(-controller.getLeftX()) * Constants.DriveConstants.kControllerDriveMultiplier;
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
}
