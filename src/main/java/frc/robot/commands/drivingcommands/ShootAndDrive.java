package frc.robot.commands.drivingcommands;

import java.util.Optional;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

/**
 * ShootAndDrive command for the robot. Drives the robot while maintaining a locked orientation
 * toward the hub/target and manages the shooter subsystem.
 *
 * @since 2026-01-15
 */
public class ShootAndDrive extends Command {

  private final Swerve swerve;
  private final Controller controller;
  private final Shooter shooter;
  private Hopper hopper;
  private Intake intake;

  private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix;
  private final PIDController headingController = Constants.DriveConstants.kHeadingController;

  /**
   * Constructor which sets swerve and controller class variables.
   *
   * @param swerve the swerve drive
   * @param shooter the shooter subsystem
   * @param hopper the hopper subsystem
   * @param intake the intake subsystem
   * @param controller the controller
   * @param interpolationMatrix the interpolation matrix for shooter speed and hood angle
   */
  public ShootAndDrive(
      Swerve swerve,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      Controller controller,
      InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix) {

    this.swerve = swerve;
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.controller = controller;
    this.interpolationMatrix = interpolationMatrix;

    addRequirements(swerve, hopper, shooter, intake);
  }

  /**
   * Calculates the target heading toward the hub or passing target, drives with locked orientation,
   * and manages shooter speed/angle interpolation and hopper feeding when ready.
   */
  @Override
  public void execute() {

    // --- alliance detection ---

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (!ally.isPresent()) {
      System.out.println("Alliance not detected!");
    }

    // --- target translation selection ---
    // if the robot is in the scoring zone for its alliance, aim at the hub
    // otherwise, calculate a passing target based on field position

    Translation2d targetTranslation;

    if (ally.get() == Alliance.Red
        && swerve.getPose().getTranslation().getX()
            > Constants.ShooterConstants.kRedAllianceXBoundary) {

      // robot is in red scoring zone, aim at red hub
      targetTranslation = Constants.ShooterConstants.kRedHubTranslation;

    } else if (ally.get() == Alliance.Blue
        && swerve.getPose().getTranslation().getX()
            < Constants.ShooterConstants.kBlueAllianceXBoundary) {

      // robot is in blue scoring zone, aim at blue hub
      targetTranslation = Constants.ShooterConstants.kBlueHubTranslation;

    } else {

      // robot is not in scoring zone, calculate a passing target
      // pick y based on which side of the field midline we're on
      double targetPoseY;

      if (this.swerve.getPose().getY() >= Constants.ShooterConstants.kFieldMidlineY) {
        targetPoseY = (Constants.ShooterConstants.kFieldMidlineY / 2) * 3;
      } else {
        targetPoseY = Constants.ShooterConstants.kFieldMidlineY / 2;
      }

      if (ally.get() == Alliance.Red) {
        targetTranslation =
            new Translation2d(Constants.ShooterConstants.kRedPassingTargetX, targetPoseY);
      } else {
        targetTranslation =
            new Translation2d(Constants.ShooterConstants.kBluePassingTargetX, targetPoseY);
      }
    }

    SmartDashboard.putNumber(
        "Distance to Hub", (swerve.getPose().getTranslation().getDistance(targetTranslation)));

    // --- heading calculation ---
    // compute angle toward target, then offset for shooter mounting position

    Pose2d currentPose = swerve.getPose();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();

    Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());

    double targetAngle =
        Math.atan2(targetTranslation.getY() - currentY, targetTranslation.getX() - currentX)
            + Math.PI / 2;

    double distance = currentPose.getTranslation().getDistance(targetTranslation);

    // offset angle to account for shooter not being centered on robot
    double angleOffset =
        Math.asin(Constants.ShooterConstants.kRobotToShooterOffsetMeters / distance);

    targetAngle = targetAngle + angleOffset;

    // --- drive with heading lock ---

    headingController.setSetpoint(targetAngle);
    double rotation = headingController.calculate(swerve.getPose().getRotation().getRadians());
    swerve.drive(translation, rotation, controller.isFieldRelative());

    // --- shooter interpolation ---
    // look up speed and hood angle from the interpolation table using distance

    double distanceToTarget = swerve.getPose().getTranslation().getDistance(targetTranslation);
    Matrix<N2, N1> interp = interpolationMatrix.get(distanceToTarget);

    double interpolatedShooterSpeed = 0.0;
    double interpolatedHoodAngle = 0.0;

    if (interp != null) {
      interpolatedShooterSpeed = interp.get(0, 0);
      interpolatedHoodAngle = interp.get(1, 0);
    }

    this.shooter.setSpeedAndAngle(interpolatedShooterSpeed, interpolatedHoodAngle);

    // --- hopper control ---
    // only feed if shooter is spun up and heading is close enough to target

    if (this.shooter.getIsReadyToShoot()
        && Math.abs(rotation) < Constants.ShooterConstants.kAimRotationTolerance) {
      hopper.setIsHopperRunning(true);
    } else {
      hopper.setIsHopperRunning(false);
    }

    // keep intake jiggling to settle notes
    intake.setJiggly(true);
  }

  /** {@inheritDoc} This command runs continuously until interrupted. */
  @Override
  public boolean isFinished() {
    return false;
  }
}
