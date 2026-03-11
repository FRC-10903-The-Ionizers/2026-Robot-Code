package frc.robot.commands.DrivingCommands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class TeleopDriveCommand extends Command {
    /**
     * TeleopDriveCommand command for the robot. Literally just drives the robot.
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Controller controller;
    private int flippedValue = 1;
    public TeleopDriveCommand(Swerve swerve, Controller controller) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve: the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.controller = controller;

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            this.flippedValue = 1;
            System.out.println("Red alliance detected, setting initial pose to 180 degrees.");
        } else {
            this.flippedValue = -1;
             System.out.println("Blue alliance detected, setting initial pose to 0 degrees.");
        }
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */
        if (controller.isFieldRelative()) {
            swerve.drive(
                new Translation2d(controller.getDriveX() * flippedValue, controller.getDriveY() * flippedValue),
                controller.getRotation(),
                true
            );
        } else {
            swerve.drive(
                new Translation2d(controller.getDriveX() * flippedValue, controller.getDriveY() * flippedValue),
                controller.getRotation(),
                controller.isFieldRelative()
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
}