package frc.robot;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivingcommands.DriveToObject;
import frc.robot.commands.drivingcommands.MovementOriented;
import frc.robot.commands.drivingcommands.ShootAndDrive;
import frc.robot.commands.drivingcommands.TrenchWarfare;
import frc.robot.statesensors.RegionHandler;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Controller;
import java.io.File;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * RobotContainer class for the robot.
 *
 * @since 2025-10
 */
public class RobotContainer {

  private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix =
      buildInterpolationMatrix();
  private static final Logger logger = LogManager.getLogger(RobotContainer.class);

  private final Swerve swerve = new Swerve();
  private final Controller controller = new Controller(Constants.kXboxControllerPort);
  private final RegionHandler regionHandler =
      new RegionHandler(new File(Filesystem.getDeployDirectory(), "misc/regions.json"));
  private final Vision vision = new Vision(swerve);
  private final Shooter shooter = new Shooter();
  // private final Vision vision = new Vision(swerve);
  private final Intake intake = new Intake(swerve);
  private final Hopper hopper = new Hopper();
  private final CommandXboxController xboxController =
      new CommandXboxController(Constants.kXboxControllerPort);

  /**
   * RobotContainer constructor for the robot.
   *
   * @since 2025-10
   */
  public RobotContainer() {
    // configure all triggers
    configureBindings();

    // send data to Elastic
    SmartDashboard.putData("SwerveField", swerve.getField());
    SmartDashboard.putData(swerve);
  }

  /**
   * Builds the interpolation matrix that maps distance-to-hub to shooter speed and hood angle pairs
   * for automatic shot adjustment.
   *
   * @return the populated interpolation matrix
   */
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

  /**
   * Gets the swerve subsystem.
   *
   * @return the swerve subsystem
   * @since 2025-10
   */
  public Swerve getSwerve() {
    return swerve;
  }

  /**
   * Gets the controller.
   *
   * @return the controller
   * @since 2025-10
   */
  public Controller getController() {
    return controller;
  }

  /**
   * Configures the bindings for the robot controller buttons.
   *
   * @since 2025-10
   */
  private void configureBindings() {
    // Bind aiming command to a button (e.g., A button)
    // xboxController.a().whileTrue(new TeleopAimDrive(swerve, controller, new
    // Pose2d(0, 0, null))); // Pass appropriate target pose (angle doesn't matter
    // for aim drive)
    xboxController.x().onTrue(intake.toggleIntakeCommand());
    xboxController
        .leftTrigger(Constants.DriveConstants.kLeftTriggerThreshold)
        .onChange(new InstantCommand(() -> controller.toggleFieldRelative()));
    xboxController
        .rightTrigger(Constants.DriveConstants.kRightTriggerThreshold)
        .whileTrue(
            new TrenchWarfare(swerve, controller));
    xboxController
        .b()
        .whileTrue(
            new ShootAndDrive(
                swerve, shooter, hopper, intake, controller, interpolationMatrix)); // Example:
    // Lock
    // to
    // 90
    // degrees
    // while
    // driving

    xboxController.y().whileTrue(new MovementOriented(swerve, controller));
    xboxController.povUp().whileTrue(new DriveToObject(swerve));


    // Bind aligning command to another button (e.g., B button)
    
    // change gear commands
    xboxController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (Swerve.getCurrentGear() + Constants.DriveConstants.kGearStep
                      < Constants.DriveConstants.kHIGH_GEAR_MULTIPLIER) {
                    Swerve.setCurrentGear(
                        Swerve.getCurrentGear() + Constants.DriveConstants.kGearStep);
                    logger.info("Shifted to HIGH gear: {}", Swerve.getCurrentGear());
                  } else {
                    Swerve.setCurrentGear(Constants.DriveConstants.kHIGH_GEAR_MULTIPLIER);
                    logger.info("Shifted to HIGH gear: {}", Swerve.getCurrentGear());
                  }
                },
                swerve));

    xboxController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (Swerve.getCurrentGear() - Constants.DriveConstants.kGearStep
                      > Constants.DriveConstants.kLOW_GEAR_MULTIPLIER) {
                    Swerve.setCurrentGear(
                        Swerve.getCurrentGear() - Constants.DriveConstants.kGearStep);
                    logger.info("Shifted to LOW gear: {}", Swerve.getCurrentGear());
                  } else {
                    Swerve.setCurrentGear(Constants.DriveConstants.kLOW_GEAR_MULTIPLIER);
                    logger.info("Shifted to LOW gear: {}", Swerve.getCurrentGear());
                  }
                },
                swerve));
  }
}
