package frc.robot.util;

import java.util.Map;
import java.util.Set;

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
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class Auto {
    /**
     * Auto class for the robot.
     * Uses choreo library to create autonomous routines.
     * @author Justin Baratta & Jake Xie
     * @since 2026-01-19
     * @todo Auto branching needs to be implemented.
     */

    private final AutoChooser autoChooser;
    private static Swerve swerve;

    public Auto(AutoFactory factory, Swerve swerve) {
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
        Auto.swerve = swerve;
        // I added the dummy command just to test things. For real autos, please make
        // routines and use their commands.
        // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

        autoChooser.addRoutine("T", this::tRoutine);
        autoChooser.addRoutine("T then Loop", this::forwardThenLoopRoutine);
        autoChooser.addRoutine("Loop", this::loopRoutine);
        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Choreo Chooser", autoChooser);
    }

    public AutoRoutine tRoutine() {
        /**
         * Test routine for the robot.
         * 
         * @args None
         * @author Justin Baratta & Jake Xie
         * @since 2026-01-19
         * @return AutoRoutine
         */
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");

        // Load the routine's trajectories
        AutoTrajectory trajectory = routine.trajectory("T.traj");

        routine.active().onTrue(
                Commands.sequence(
                        trajectory.resetOdometry(),
                        cmdWithAccuracy(
                                trajectory,
                                Units.Seconds.of(5),
                                Units.Inches.of(4.0))));

        return routine;
    }

    public AutoRoutine loopRoutine() {
        /**
         * Test loop routine for the robot - drives robot in small loop.
         * 
         * @args None
         * @author Justin Baratta & Jake Xie
         * @since 2026-01-19
         * @return AutoRoutine
         */
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Loop");

        AutoTrajectory loopTrajectory = routine.trajectory("Loop.traj");

        routine.active().onTrue(
                Commands.sequence(
                        loopTrajectory.resetOdometry(),
                        cmdWithAccuracy(
                                loopTrajectory,
                                Units.Seconds.of(5),
                                Units.Inches.of(4.0))));

        return routine;
    }

    public AutoRoutine forwardThenLoopRoutine() {
        /**
         * Moves forward, then loops around the field.
         * 
         * @args None
         * @author Justin Baratta & Jake Xie
         * @since 2026-01-19
         * @return AutoRoutine
         */
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("ForwardThenLoop");

        AutoTrajectory forwardTrajectory = routine.trajectory("Forward.traj");
        AutoTrajectory loopTrajectory = routine.trajectory("Loop.traj");

        routine.active().onTrue(
                Commands.sequence(
                        forwardTrajectory.resetOdometry(),
                        cmdWithAccuracy(
                                forwardTrajectory,
                                Units.Seconds.of(5),
                                Units.Inches.of(4.0)),
                        cmdWithAccuracy(
                                loopTrajectory,
                                Units.Seconds.of(5),
                                Units.Inches.of(4.0))));

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

    private static boolean isFinished(AutoTrajectory trajectory, Distance epsilonDist) {
        /**
         * Checks if the trajectory is finished, based on whether translation and rotation are finished.
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

    private static boolean translationIsFinished(AutoTrajectory trajectory, Distance epsilonDist) {
        /**
         * Checks if the translation is finished. Used in {@link #isFinished(AutoTrajectory, Distance)}
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

        return currentPose.getTranslation().getDistance(finalPose.getTranslation()) < epsilonDist.in(Units.Meters);
    }

    private static boolean rotationIsFinished(AutoTrajectory trajectory) {
        /**
         * Checks if the rotation is finished. Used in {@link #isFinished(AutoTrajectory, Distance)}
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
                Math.abs(currentPose.getRotation().minus(finalPose.getRotation()).getRadians())) < epsilonAngle
                        .in(Units.Radians);
    }

    public static Command cmdWithAccuracy(AutoTrajectory trajectory, Time timeout, Distance epsilonDist) {
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
                        () -> isFinished(trajectory, epsilonDist)),
                Set.of(swerve))
                .withTimeout(trajectory.getRawTrajectory().getTotalTime() + timeout.in(Units.Seconds))
                .finallyDo(() -> swerve.drive(new Translation2d(), 0, false));
    }
}