package frc.robot;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Auto;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private final Timer timer = new Timer();
    private AutoFactory autoFactory;
    private Auto auto;

    private Swerve swerve;
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        swerve = robotContainer.getSwerve();

        autoFactory = new AutoFactory(
            swerve::getPose, // A function that returns the current robot pose
            swerve::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerve::followTrajectory, // T`he drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerve // The drive subsystem
        );

        auto = new Auto(autoFactory);
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
        System.out.println("Auto started...");
        timer.restart();
        // if (autoCommand != null) {
        //     autoCommand.schedule();
        // }
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.controllerDrive();
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
