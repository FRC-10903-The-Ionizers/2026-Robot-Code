package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import frc.robot.Constants.Vision;
import edu.wpi.first.math.controller.PIDController;

public class DriveToObject extends Command {
    /**
    * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final PIDController controller;

    public DriveToObject(Swerve swerve, PIDController controller) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve: the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
    /**
     * Method that drives the swerves
     */
    try {
    // see available cameras

        double[] dataFromPython = LimelightHelpers.getPythonScriptData("limelight-obj");
        System.out.println("Data from Python: " + java.util.Arrays.toString(dataFromPython));
        // index 0 is x value of brightest point, we want to center that
        double angle_diff = (dataFromPython[0] - Vision.kObjectCameraPixelsHorizontal / 2) / Vision.kObjectCameraPixelsHorizontal * Vision.kObjectCameraFovHorizontal;
        System.out.println("Angle difference: " + angle_diff);
        // feed into pid controller to get rotation output
        double rotationOutput = controller.calculate(angle_diff, 0);
      
     
        if (dataFromPython[2] > Vision.kObjectDetectionThreshold) {
            swerve.drive(new Translation2d(1, 0), rotationOutput/10, false);
        }
        else {
            swerve.drive(new Translation2d(0, 0), 1, false);
        }
    } catch (Exception e) {
        System.out.println("Error in DriveToObject command: " + e.getMessage());
    }

    }


    public boolean isFinished() {
        return false; // Runs continuously
    }
}