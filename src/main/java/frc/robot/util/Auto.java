package frc.robot.util;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DrivingCommands.DriveToPose;
import frc.robot.statesensors.RegionHandler;
import frc.robot.subsystems.Swerve;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Auto class for the robot. Uses choreo library to create autonomous routines.
 *
 * @since 2026-01-19
 * @todo Auto branching needs to be implemented.
 */
public class Auto {

  private final AutoChooser branchingAutoChooser;
  private static Swerve swerve;
  private final RegionHandler regionHandler;
  private static final Logger logger = LogManager.getLogger(Auto.class);

  /**
   * Auto constructor for the robot.
   *
   * @param factory the auto factory
   * @param swerve the swerve subsystem
   * @param regionHandler the region handler
   * @since 2026-01-19
   */
  public Auto(AutoFactory factory, Swerve swerve, RegionHandler regionHandler) {
    // Make something to display on the GUI that allows the user to select the
    // autonomous they want to run
    branchingAutoChooser = new AutoChooser();
    this.regionHandler = regionHandler;
    Auto.swerve = swerve;

    // I added the dummy command just to test things. For real autos, please make
    // routines and use their commands.
    // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

    branchingAutoChooser.addRoutine("Branching Auto", this::branchingAutoRoutine);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", branchingAutoChooser);
    SmartDashboard.putBoolean("Go Under Trench", false);
    // SmartDashboard.getBoolean("Go Under Trench", false);
  }

  /**
   * Creates the branching autonomous routine. Selects trajectories based on alliance color from
   * game data and the robot's starting region, with conditional branching for trench vs bump paths.
   *
   * @return the configured auto routine
   */
  public AutoRoutine branchingAutoRoutine() {
    AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
    String gameData = DriverStation.getGameSpecificMessage();
    // Wrap everything in a deferred command so pose is read at execution time

    if (gameData.length() < 1) {
      logger.error("Nothing in gameData in DS!");
      throw new Error();
    }

    switch (gameData.charAt(0)) {
      case 'B':
        routine
            .active()
            .onTrue(
                Commands.defer(
                    () -> {
                      Pose2d currentPose = swerve.getPose();

                      if (regionHandler.inRegion("region1", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Blue1");
                        AutoTrajectory trenchTrajectory = routine.trajectory("BlueTopTrench");
                        AutoTrajectory bumpTrajectory = routine.trajectory("BlueTopBump");

                        return Commands.sequence(
                            Commands.print("In region 1"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, trenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(trenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, bumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else if (regionHandler.inRegion("region2", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Blue2Top");
                        AutoTrajectory topTrenchTrajectory = routine.trajectory("BlueTopTrench");
                        AutoTrajectory topBumpTrajectory = routine.trajectory("BlueTopBump");

                        return Commands.sequence(
                            Commands.print("In region 2"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, topTrenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(topTrenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, topBumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(topBumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else if (regionHandler.inRegion("region3", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Blue2Bottom");
                        AutoTrajectory bottomTrenchTrajectory =
                            routine.trajectory("BlueBottomTrench");
                        AutoTrajectory bottomBumpTrajectory = routine.trajectory("BlueBottomBump");

                        return Commands.sequence(
                            Commands.print("In region 3"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve,
                                        bottomTrenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bottomTrenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve,
                                        bottomBumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bottomBumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else if (regionHandler.inRegion("region4", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Blue3");
                        AutoTrajectory bottomTrenchTrajectory =
                            routine.trajectory("BlueBottomTrench");
                        AutoTrajectory bottomBumpTrajectory = routine.trajectory("BlueBottomBump");

                        return Commands.sequence(
                            Commands.print("In region 4"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve,
                                        bottomTrenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bottomTrenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve,
                                        bottomBumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bottomBumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else {
                        AutoTrajectory trajectory = routine.trajectory("Safe");
                        return Commands.sequence(
                            Commands.print("Default Routine chosen"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory));
                      }
                    },
                    Set.of(swerve)));
        return routine;

      case 'R':
        routine
            .active()
            .onTrue(
                Commands.defer(
                    () -> {
                      Pose2d currentPose = swerve.getPose();

                      if (regionHandler.inRegion("region5", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Red1");
                        AutoTrajectory trenchTrajectory = routine.trajectory("RedTopTrench");
                        AutoTrajectory bumpTrajectory = routine.trajectory("RedTopBump");

                        return Commands.sequence(
                            Commands.print("In region 5"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, trenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(trenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, bumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else if (regionHandler.inRegion("region6", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Red2Top");
                        AutoTrajectory trenchTrajectory = routine.trajectory("RedTopTrench");
                        AutoTrajectory bumpTrajectory = routine.trajectory("RedTopBump");

                        return Commands.sequence(
                            Commands.print("In region 6"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, trenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(trenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, bumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else if (regionHandler.inRegion("region7", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Red2Bottom");
                        AutoTrajectory trenchTrajectory = routine.trajectory("RedBottomTrench");
                        AutoTrajectory bumpTrajectory = routine.trajectory("RedTopTrench");

                        return Commands.sequence(
                            Commands.print("In region 7"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, trenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(trenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, bumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else if (regionHandler.inRegion("region8", currentPose)) {
                        AutoTrajectory trajectory = routine.trajectory("Red3");
                        AutoTrajectory trenchTrajectory = routine.trajectory("RedBottomTrench");
                        AutoTrajectory bumpTrajectory = routine.trajectory("RedBottomBump");

                        return Commands.sequence(
                            Commands.print("In region 8"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory),
                            new ConditionalCommand(
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, trenchTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(trenchTrajectory)),
                                Commands.sequence(
                                    new DriveToPose(
                                        swerve, bumpTrajectory.getInitialPose().orElseThrow()),
                                    cmdWithAccuracy(bumpTrajectory)),
                                this::getIsGoingUnderTrench));

                      } else {
                        AutoTrajectory trajectory = routine.trajectory("Safe");
                        return Commands.sequence(
                            Commands.print("Default Routine chosen"),
                            trajectory.resetOdometry(),
                            cmdWithAccuracy(trajectory));
                      }
                    },
                    Set.of(swerve)));
        return routine;
      default:
        logger.info("Not the right gamedata in gamedata section... needs to be 'B' or 'R'");
        throw new Error();
    }
  }

  /**
   * Gets the selected auto command from the auto chooser.
   *
   * @return the selected auto command
   * @since 2026-01-19
   */
  public Command getAutoCommand() {
    return branchingAutoChooser.selectedCommand();
  }

  /**
   * Returns whether the driver has selected the "go under trench" option on SmartDashboard, used
   * for branching auto decisions.
   *
   * @return true if the robot should go under the trench
   */
  public boolean getIsGoingUnderTrench() {
    return SmartDashboard.getBoolean("Go Under Trench", false);
  }

  /**
   * Checks if the trajectory is finished, based on whether translation and rotation are finished.
   * This code was inspired by Citrus Circuits' 2025 code.
   *
   * @param trajectory the auto trajectory
   * @param epsilonDist the distance epsilon
   * @return true if the trajectory is finished
   * @since 2026-01-19
   */
  private static boolean isFinished(AutoTrajectory trajectory, Distance epsilonDist) {
    boolean translationCompleted = translationIsFinished(trajectory, epsilonDist);
    boolean rotationCompleted = rotationIsFinished(trajectory);

    SmartDashboard.putBoolean("Choreo/Translation Completed", translationCompleted);
    SmartDashboard.putBoolean("Choreo/Rotation Completed", rotationCompleted);

    return translationCompleted && rotationCompleted;
  }

  /**
   * Checks if the translation is finished. Used in {@link #isFinished(AutoTrajectory, Distance)}.
   * This code was inspired by Citrus Circuits' 2025 code.
   *
   * @param trajectory the auto trajectory
   * @param epsilonDist the distance epsilon
   * @return true if translation is finished
   * @since 2026-01-19
   */
  private static boolean translationIsFinished(AutoTrajectory trajectory, Distance epsilonDist) {
    Pose2d currentPose = swerve.getPose();
    Pose2d finalPose = trajectory.getFinalPose().get();

    SmartDashboard.putNumber(
        "Choreo/Distance Away Inches",
        currentPose.getTranslation().getDistance(finalPose.getTranslation())
            * Constants.kMetersToInches);

    return currentPose.getTranslation().getDistance(finalPose.getTranslation())
        < epsilonDist.in(Units.Meters);
  }

  /**
   * Checks if the rotation is finished. Used in {@link #isFinished(AutoTrajectory, Distance)}. This
   * code was inspired by Citrus Circuits' 2025 code.
   *
   * @param trajectory the auto trajectory
   * @return true if rotation is finished
   * @since 2026-01-19
   */
  private static boolean rotationIsFinished(AutoTrajectory trajectory) {
    Pose2d currentPose = swerve.getPose();
    Pose2d finalPose = trajectory.getFinalPose().get();
    Angle epsilonAngle = AutoConstants.kAutoAngleEpsilon;

    return MathUtil.angleModulus(
            Math.abs(currentPose.getRotation().minus(finalPose.getRotation()).getRadians()))
        < epsilonAngle.in(Units.Radians);
  }

  /**
   * Creates a command with accuracy. Used in basically all the routines. This code was inspired by
   * Citrus Circuits' 2025 code.
   *
   * @param trajectory the auto trajectory
   * @return the command with accuracy checks
   * @since 2026-01-19
   */
  public static Command cmdWithAccuracy(AutoTrajectory trajectory) {
    return Commands.defer(
            () ->
                new FunctionalCommand(
                    trajectory.cmd()::initialize,
                    trajectory.cmd()::execute,
                    trajectory.cmd()::end,
                    () -> isFinished(trajectory, Constants.AutoConstants.kEpsilonDist)),
            Set.of(swerve))
        .withTimeout(
            trajectory.getRawTrajectory().getTotalTime()
                + Constants.AutoConstants.kTimeout.in(Units.Seconds))
        .finallyDo(() -> swerve.drive(new Translation2d(), 0, false));
  }
}
