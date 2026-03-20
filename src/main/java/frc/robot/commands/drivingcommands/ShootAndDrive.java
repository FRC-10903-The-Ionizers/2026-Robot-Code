package frc.robot.commands.drivingcommands;

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
import java.util.Optional;

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
  private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix;

  private final PIDController headingController = Constants.DriveConstants.kHeadingController;
  private Intake intake;

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
    // Get target translation
    Optional<Alliance> ally = DriverStation.getAlliance();

    if (!ally.isPresent()) {
      System.out.println("Alliance not detected!");
    }
    Translation2d targetTranslation;
    if (ally.get() == Alliance.Red
        && swerve.getPose().getTranslation().getX()
            > Constants.ShooterConstants.kRedAllianceXBoundary) {
      targetTranslation = Constants.ShooterConstants.kRedHubTranslation;
    } else if (ally.get() == Alliance.Blue
        && swerve.getPose().getTranslation().getX()
            < Constants.ShooterConstants.kBlueAllianceXBoundary) {
      targetTranslation = Constants.ShooterConstants.kBlueHubTranslation;
    } else {
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
    double distanceToTarget = swerve.getPose().getTranslation().getDistance(targetTranslation);

    // Drivetrain stuff
    Pose2d currentPose = swerve.getPose();
    double currentX = currentPose.getX();
    double currentY = currentPose.getY();
    Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
    double target_angle =
        Math.atan2(targetTranslation.getY() - currentY, targetTranslation.getX() - currentX)
            + Math.PI / 2;

    double distance = currentPose.getTranslation().getDistance(targetTranslation);

    double angleOffset =
        Math.asin(Constants.ShooterConstants.kRobotToShooterOffsetMeters / distance);
    target_angle = target_angle + angleOffset;
    headingController.setSetpoint(target_angle);
    double rotation = headingController.calculate(swerve.getPose().getRotation().getRadians());
    swerve.drive(translation, rotation, controller.isFieldRelative());

    // With distance start getting shooting stuff set
    Matrix<N2, N1> interp = interpolationMatrix.get(distanceToTarget);
    double interpolatedShooterSpeed = 0.0;
    double interpolatedHoodAngle = 0.0;
    if (interp != null) {
      interpolatedShooterSpeed = interp.get(0, 0);
      interpolatedHoodAngle = interp.get(1, 0);
    }

    this.shooter.setSpeedAndAngle(interpolatedShooterSpeed, interpolatedHoodAngle);
    if ((this.shooter.getIsReadyToShoot())
        && Math.abs(rotation) < Constants.ShooterConstants.kAimRotationTolerance) {
      hopper.setIsHopperRunning(true);
    } else {
      hopper.setIsHopperRunning(false);
    }

    intake.setJiggly(true);
  }

  /** {@inheritDoc} This command runs continuously until interrupted. */
  @Override
  public boolean isFinished() {
    return false; // Runs continuously
  }
}
