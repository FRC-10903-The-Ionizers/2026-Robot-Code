package frc.robot.util;

import java.util.Set;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DrivingCommands.ShootAndDrive;
import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Auto {
    /**
     * Auto class for the robot.
     * Uses choreo library to create autonomous routines.
     * 
     * @author Justin Baratta & Jake Xie
     * @since 2026-01-19
     * @todo Auto branching needs to be implemented.
     */

    private final AutoChooser autoChooser;
    private final Swerve swerve;
    private final RegionHandler regionHandler;
    private static final Logger logger = LogManager.getLogger(Auto.class);
    private final AutoFactory autoFactory = Constants.AutoConstants.kAutoFactory;
    private final Controller controller = new Controller(Constants.kXboxControllerPort);
    private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix = buildInterpolationMatrix();
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    private final double timeout = 3.3;

    public Auto(AutoFactory factory, RobotContainer container, RegionHandler regionHandler) {
        /**
         * Auto constructor for the robot.
         * 
         * @args AutoFactory factory, Swerve swerve
         * @author Justin Baratta & Jake Xie
         * @since 2026-01-19
         * @return void
         */
        // Make something to display on the GUI that allows the user to select the
        // autonomous they want to run
        autoChooser = new AutoChooser();
        this.regionHandler = regionHandler;
        this.swerve = container.getSwerve();
        this.shooter = container.getShooter();
        this.hopper = container.getHopper();
        this.intake = container.getIntake();

        autoFactory.bind("Shoot", new ShootAndDrive(swerve, shooter, hopper, controller, interpolationMatrix, timeout));
        autoFactory.bind("ToggleIntake", intake.toggleIntakeCommand());

        // I added the dummy command just to test things. For real autos, please make
        // routines and use their commands.
        // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

        autoChooser.addRoutine("Red Left", this::redLeftAutoRoutine);
        autoChooser.addRoutine("Red Right", this::redRightAutoRoutine);
        autoChooser.addRoutine("Red Middle Top", this::redMiddleTopAutoRoutine);
        autoChooser.addRoutine("Red Middle Bottom", this::redMiddleBottomAutoRoutine);
        autoChooser.addRoutine("Blue Right", this::blueRightAutoRoutine);
        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // SmartDashboard.getBoolean("Go Under Trench", false);
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

    public AutoRoutine redLeftAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("RedLeft.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Red Left Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));
        return routine;
    }

    public AutoRoutine redRightAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("RedRight.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Red Right Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public AutoRoutine redMiddleTopAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("RedMiddleTop.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Red Middle Top Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public AutoRoutine redMiddleBottomAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("RedMiddleBottom.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Red Middle Bottom Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public AutoRoutine blueLeftAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("BlueLeft.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Blue Left Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public AutoRoutine blueRightAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("BlueRight.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Blue Right Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public AutoRoutine blueMiddleTopAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("BlueMiddleTop.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Blue Middle Top Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public AutoRoutine blueMiddleBottomAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = routine.trajectory("BlueMiddleBottom.traj");

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Blue Middle Bottom Auto Chosen"),
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(trajectory)));

        return routine;
    }

    public Command getAutoCommand() {
        /**
         * Gets the selected auto command from the auto chooser.
         * 
         * @args None
         * @author Justin Baratta & Jake Xie
         * @since 2026-01-19
         * @return Command
         */
        return autoChooser.selectedCommand();
    }

    private boolean isFinished(AutoTrajectory trajectory, Distance epsilonDist) {
        /**
         * Checks if the trajectory is finished, based on whether translation and
         * rotation are finished.
         * This code was inspired by Citrus Circuits' 2025 code.
         * 
         * @args AutoTrajectory trajectory, Distance epsilonDist
         * @author Justin Baratta & Jake Xie, From 2025 Citrus Circuits Team
         * @since 2026-01-19
         * @return boolean
         */
        boolean translationCompleted = translationIsFinished(trajectory, epsilonDist);
        boolean rotationCompleted = rotationIsFinished(trajectory);

        SmartDashboard.putBoolean("Choreo/Translation Completed", translationCompleted);
        SmartDashboard.putBoolean("Choreo/Rotation Completed", rotationCompleted);

        return translationCompleted && rotationCompleted;
    }

    private boolean translationIsFinished(AutoTrajectory trajectory, Distance epsilonDist) {
        /**
         * Checks if the translation is finished. Used in
         * {@link #isFinished(AutoTrajectory, Distance)}
         * This code was inspired by Citrus Circuits' 2025 code.
         * 
         * @args AutoTrajectory trajectory, Distance epsilonDist
         * @author Justin Baratta & Jake Xie, From 2025 Citrus Circuits Team
         * @since 2026-01-19
         * @return boolean
         */
        Pose2d currentPose = swerve.getPose();
        Pose2d finalPose = trajectory.getFinalPose().get();

        SmartDashboard.putNumber(
                "Choreo/Distance Away Inches",
                currentPose.getTranslation().getDistance(finalPose.getTranslation()) * 39.37);

        return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist
                .in(Units.Meters);
    }

    private boolean rotationIsFinished(AutoTrajectory trajectory) {
        /**
         * Checks if the rotation is finished. Used in
         * {@link #isFinished(AutoTrajectory, Distance)}
         * This code was inspired by Citrus Circuits' 2025 code.
         * 
         * @args AutoTrajectory trajectory
         * @author Justin Baratta & Jake Xie, From 2025 Citrus Circuits Team
         * @since 2026-01-19
         * @return boolean
         */
        Pose2d currentPose = swerve.getPose();
        Pose2d finalPose = trajectory.getFinalPose().get();
        Angle epsilonAngle = AutoConstants.kAutoAngleEpsilon;

        return MathUtil.angleModulus(
                Math.abs(currentPose.getRotation().minus(finalPose.getRotation())
                        .getRadians())) < epsilonAngle
                                .in(Units.Radians);
    }

    public Command cmdWithAccuracy(AutoTrajectory trajectory) {
        /**
         * Creates a command with accuracy. Used in basically all the routines.
         * This code was inspired by Citrus Circuits' 2025 code.
         * 
         * @args AutoTrajectory trajectory, Time timeout, Distance epsilonDist
         * @author Justin Baratta & Jake Xie, 2025 Citrus Circuits Team
         * @since 2026-01-19
         * @return Command
         */
        return Commands.defer(
                () -> new FunctionalCommand(
                        trajectory.cmd()::initialize,
                        trajectory.cmd()::execute,
                        trajectory.cmd()::end,
                        () -> isFinished(trajectory, Constants.AutoConstants.kEpsilonDist)),
                Set.of(swerve))
                .withTimeout(trajectory.getRawTrajectory().getTotalTime()
                        + Constants.AutoConstants.kTimeout.in(Units.Seconds))
                .finallyDo(() -> {
                    swerve.drive(new Translation2d(), 0, false);
                });
    }
}