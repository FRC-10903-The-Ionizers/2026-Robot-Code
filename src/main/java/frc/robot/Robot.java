package frc.robot;

import choreo.auto.AutoFactory;
import java.io.File;
import java.util.Optional;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.config.Configurator;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DrivingCommands.TeleopDriveCommand;
import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.Controller;

public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private final Timer timer = new Timer();
    private Auto auto;
    private RegionHandler regionHandler;
    private final static Logger logger = LogManager.getLogger(Robot.class);

    public Robot() {
        /**
         * Robot constructor for the robot.
         * 
         * @args None
         * @author Max Clemetson, Jake Xie, Justin Baratta, Siddhartha Hiremath
         * @since 2025-10
         * @return void
         */

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

        Constants.AutoConstants.kAutoFactory = autoFactory;

        auto = new Auto(autoFactory, robotContainer, regionHandler);
        logger.debug("Auto initialized!");
        RobotModeTriggers.autonomous().onTrue(auto.getAutoCommand());
        // Optionally disable the joystick connection warning
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        /**
         * Schedules the autonomous command, runs automatically when the robot is in
         * autonomous mode.
         * 
         * @args None
         * @author Jake Xie & Justin Baratta
         * @since 2026-01
         * @return void
         */
        timer.restart();
        Command autoCommand = auto.getAutoCommand();

        if (autoCommand != null) {
            autoCommand.schedule();
        }

    }

    @Override
    public void autonomousPeriodic() {
        /**
         * @todo Remove this unless needed
         */
    }

    @Override
    public void teleopInit() {
        /**
         * Initializes the teleop mode.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        Swerve swerve = robotContainer.getSwerve();
        Controller controller = robotContainer.getController();
        TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(swerve, controller);

        swerve.setDefaultCommand(teleopDriveCommand);
    }

    @Override
    public void teleopPeriodic() {
        /**
         * @todo Remove this unless needed
         */
    }

    @Override
    public void teleopExit() {
        /**
         * Exits the teleop mode, runs when the robot is no longer in teleop mode.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        Swerve swerve = robotContainer.getSwerve();
        swerve.removeDefaultCommand();
    }

    private boolean isRedAlliance() {
        /**
         * Checks if the robot is on the red alliance.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return boolean
         */

         Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            return (alliance.get() == Alliance.Red) ? true : false;
        }  else {
            throw new Error("Set Alliance is DS pls");
        }
    }
}
