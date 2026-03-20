package frc.robot;

import java.io.File;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DrivingCommands.TeleopDriveCommand;
import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.Controller;

/**
 * Robot class for the robot.
 * 
 * @since 2025-10
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private final Timer timer = new Timer();
    private Auto auto;
    private RegionHandler regionHandler;
    private final static Logger logger = LogManager.getLogger(Robot.class);

    /**
     * Robot constructor for the robot.
     * 
     * @since 2025-10
     */
    public Robot() {
        // Save network tables data to disk
        DataLogManager.start();

        logger.debug("Robot initializing");

        robotContainer = new RobotContainer();
        regionHandler = new RegionHandler(new File(Filesystem.getDeployDirectory(), "misc/regions.json"));
        Swerve swerve = robotContainer.getSwerve();

        AutoFactory autoFactory = new AutoFactory(
                swerve::getPose, // A function that returns the current robot pose
                swerve::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
                swerve::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                swerve // The drive subsystem
        );

        autoFactory.bind("Yo", Commands.print("At Yo"));

        Constants.AutoConstants.kAutoFactory = autoFactory;

        auto = new Auto(autoFactory, swerve, regionHandler);
        logger.debug("Auto initialized!");
        RobotModeTriggers.autonomous().onTrue(auto.getAutoCommand());
        // Optionally disable the joystick connection warning
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * Runs the command scheduler every robot loop iteration.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedules the autonomous command, runs automatically when the robot is in
     * autonomous mode.
     * 
     * @since 2026-01
     */
    @Override
    public void autonomousInit() {
        timer.restart();
        Command autoCommand = auto.getAutoCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }

    }

    /**
     * Called periodically during autonomous mode. Currently a no-op.
     *
     * @todo Remove this unless needed
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * Initializes the teleop mode.
     * 
     * @since 2025-10
     */
    @Override
    public void teleopInit() {
        Swerve swerve = robotContainer.getSwerve();
        Controller controller = robotContainer.getController();
        TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(swerve, controller);

        swerve.setDefaultCommand(teleopDriveCommand);
    }

    /**
     * Called periodically during teleop mode. Currently a no-op.
     *
     * @todo Remove this unless needed
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * Exits the teleop mode, runs when the robot is no longer in teleop mode.
     * 
     * @since 2025-10
     */
    @Override
    public void teleopExit() {
        Swerve swerve = robotContainer.getSwerve();
        swerve.removeDefaultCommand();
    }

    /**
     * Checks if the robot is on the red alliance.
     * 
     * @return true if the robot is on the red alliance
     * @since 2025-10
     */
    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
