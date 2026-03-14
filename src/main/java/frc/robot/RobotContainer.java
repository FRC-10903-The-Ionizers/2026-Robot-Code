package frc.robot;

import java.io.File;
import java.util.Optional;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DrivingCommands.ShootAndDrive; // Assuming this command exists for aiming
import frc.robot.commands.DrivingCommands.DriveToPose;
import frc.robot.commands.DrivingCommands.ShootAndDrive; // Assuming this command exists for aiming
import frc.robot.commands.DrivingCommands.DriveToObject;
import frc.robot.commands.DrivingCommands.MovementOriented;
import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Controller;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class RobotContainer {

    /**
     * RobotContainer class for the robot.
     * 
     * @author Max Clemetson, Jake Xie, Justin Baratta, Siddhartha Hiremath
     * @since 2025-10
     */

    private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix = buildInterpolationMatrix();
    private final static Logger logger = LogManager.getLogger(RobotContainer.class);

    private final Swerve swerve = new Swerve();
    private final Controller controller = new Controller(Constants.kXboxControllerPort);
    private final RegionHandler regionHandler = new RegionHandler(
            new File(Filesystem.getDeployDirectory(), "misc/regions.json"));
    private final Vision vision = new Vision(swerve);
    private final Shooter shooter = new Shooter();
    // private final Vision vision = new Vision(swerve);
    private final Intake intake = new Intake(swerve);
    private final Hopper hopper = new Hopper();
    private final CommandXboxController xboxController = new CommandXboxController(Constants.kXboxControllerPort);

    public RobotContainer() {
        /**
         * RobotContainer constructor for the robot.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        // configure all triggers
        configureBindings();

        // send data to Elastic
        SmartDashboard.putData("SwerveField", swerve.getField());
        SmartDashboard.putData(swerve);
    }

    private static InterpolatingMatrixTreeMap<Double, N2, N1> buildInterpolationMatrix() {
        InterpolatingMatrixTreeMap<Double, N2, N1> map = new InterpolatingMatrixTreeMap<>();
        double[] dists = Constants.ShooterConstants.kDistancesToHub;
        double[] speeds = Constants.ShooterConstants.kShooterSpeeds;
        double[] angles = Constants.ShooterConstants.kHoodAngles;
        int len = Math.min(dists.length, Math.min(speeds.length, angles.length));
        for (int i = 0; i < len; i++) {
            map.put(dists[i], VecBuilder.fill(speeds[i], angles[i]));
        }
        return map;
    }

    // Remove controllerDrive() as it's replaced by commands

    public Swerve getSwerve() {
        /**
         * Gets the swerve subsystem.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return Swerve swerve
         */
        return swerve;
    }

    public Controller getController() {
        /**
         * Gets the controller.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return Controller controller
         */
        return controller;
    }

    private void configureBindings() {
        /**
         * Configures the bindings for the robot, add to this when u have more buttons
         * that do stuff.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        // Bind aiming command to a button (e.g., A button)
        //xboxController.a().whileTrue(new TeleopAimDrive(swerve, controller, new Pose2d(0, 0, null))); // Pass appropriate target pose (angle doesn't matter for aim drive)
        xboxController.x().onTrue(intake.toggleIntakeCommand());
        xboxController.leftTrigger(0.25).onChange(new InstantCommand(()->controller.toggleFieldRelative()));
        xboxController.b().whileTrue(new ShootAndDrive(swerve, shooter, hopper, intake, controller, interpolationMatrix)); // Example: Lock to 90 degrees while driving

        // Bind aligning command to another button (e.g., B button)

        // xboxController.b().whileTrue(new DriveToObject(swerve, new
        // PIDController(Constants.DriveConstants.kHeadingP,
        // Constants.DriveConstants.kHeadingI, Constants.DriveConstants.kHeadingD)));

        // xboxController.y().onTrue(Commands.runOnce(() ->
        // Swerve.toggleIsLockedPosition(), swerve));

        // xboxController.a().onTrue(new InstantCommand(() -> shooter.setHoodAngle(2), shooter));
        // Bind go to point test command to the D pad
        //xboxController.povLeft().onTrue(new DriveToPose(swerve, new Pose2d(-1.5, 0, Rotation2d.kZero)));
        //xboxController.povRight().onTrue(new DriveToPose(swerve, new Pose2d(1.5, 0, Rotation2d.kZero)));
        //xboxController.povDown().onTrue(new DriveToPose(swerve, new Pose2d(0, -1.5, Rotation2d.kZero)));

        // change gear commands
        xboxController.rightBumper().onTrue(Commands.runOnce(() -> {
            if (Swerve.getCurrentGear() + 1.0 < Constants.DriveConstants.kHIGH_GEAR_MULTIPLIER) {
                Swerve.setCurrentGear(Swerve.getCurrentGear() + 1.0);
                logger.info("Shifted to HIGH gear: {}", Swerve.getCurrentGear());
            } else {
                Swerve.setCurrentGear(Constants.DriveConstants.kHIGH_GEAR_MULTIPLIER);
                logger.info("Shifted to HIGH gear: {}", Swerve.getCurrentGear());
            }
        }, swerve));

        xboxController.leftBumper().onTrue(Commands.runOnce(() -> {
            if (Swerve.getCurrentGear() - 1.0 > Constants.DriveConstants.kLOW_GEAR_MULTIPLIER) {
                Swerve.setCurrentGear(Swerve.getCurrentGear() - 1);
                logger.info("Shifted to LOW gear: {}", Swerve.getCurrentGear());
            } else {
                Swerve.setCurrentGear(Constants.DriveConstants.kLOW_GEAR_MULTIPLIER);
                logger.info("Shifted to LOW gear: {}", Swerve.getCurrentGear());
            }
        }, swerve));

        
    }
}