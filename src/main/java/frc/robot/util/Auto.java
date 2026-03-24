package frc.robot.util;

import static frc.robot.trajectories.ChoreoTraj.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivingcommands.ShootAndDrive;
import frc.robot.statesensors.RegionHandler;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * Auto class for the robot. Uses choreo library to create autonomous routines.
 *
 * @since 2026-01-19
 * @todo Auto branching needs to be implemented.
 */
public class Auto {

    private final AutoChooser autoChooser;
    private final Swerve swerve;

    // private static final Logger logger = LogManager.getLogger(Auto.class);

    private final AutoFactory autoFactory = Constants.AutoConstants.kAutoFactory;
    private final Controller controller = new Controller(Constants.kXboxControllerPort);
    private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix = buildInterpolationMatrix();
    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

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
        this.swerve = container.getSwerve();
        this.shooter = container.getShooter();
        this.hopper = container.getHopper();
        this.intake = container.getIntake();

        autoFactory.bind("ToggleIntake", intake.toggleIntakeCommand());

        // I added the dummy command just to test things. For real autos, please make
        // routines and use their commands.
        // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

        autoChooser.addRoutine("Right", this::rightAutoRoutine);
        autoChooser.addRoutine("Left", this::leftAutoRoutine);

        autoChooser.addRoutine("Safe Auto", this::safeAutoRoutine);
        autoChooser.addRoutine("Outpost", this::outpost);

        autoChooser.addRoutine("Cheesy Poofs Right", this::cheesyPoofsRight);
        autoChooser.addRoutine("Cheesy Poofs Left", this::cheesyPoofsLeft);

        autoChooser.addRoutine("Citrus Right", this::citrusRight);
        autoChooser.addRoutine("Citrus Left", this::citrusLeft);
        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

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

    public AutoRoutine safeAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        routine.active().onTrue(
                new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                        20));

        return routine;
    }

    public AutoRoutine outpost() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = Outpost$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = Outpost$1.asAutoTraj(routine);

        routine.active().onTrue(Commands.sequence(
                cmdWithAccuracy(trajectory1),
                Commands.waitSeconds(Constants.AutoConstants.kAutoShooterTimeout),
                cmdWithAccuracy(trajectory2),
                new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                        Constants.AutoConstants.kAutoShooterTimeout)));
        return routine;
    }

    public AutoRoutine rightAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = Right$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = Right$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Right Auto Chosen"),
                        cmdWithAccuracy(trajectory1),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout),
                        cmdWithAccuracy(trajectory2),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout)));
        return routine;
    }

    public AutoRoutine leftAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = Left$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = Left$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Left Auto Chosen"),
                        cmdWithAccuracy(trajectory1),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout),
                        cmdWithAccuracy(trajectory2),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout)));
        return routine;
    }

    public AutoRoutine cheesyPoofsRight() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = CheesyPoofsRight$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = CheesyPoofsRight$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Cheesy Poofs Right Auto Chosen"),
                        cmdWithAccuracy(trajectory1),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout),
                        cmdWithAccuracy(trajectory2),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout)));
        return routine;
    }

    public AutoRoutine cheesyPoofsLeft() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = CheesyPoofsLeft$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = CheesyPoofsLeft$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Cheesy Poofs Left Auto Chosen"),
                        cmdWithAccuracy(trajectory1),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout),
                        cmdWithAccuracy(trajectory2),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller,
                                interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout)));
        return routine;

    }

    public AutoRoutine citrusRight() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = CitrusRight$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = CitrusRight$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Citrus Right Auto Chosen"),
                        cmdWithAccuracy(trajectory1),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout),
                        cmdWithAccuracy(trajectory2),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout)));

        return routine;
    }

    public AutoRoutine citrusLeft() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = CitrusLeft$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = CitrusLeft$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Citrus Left Auto Chosen"),
                        cmdWithAccuracy(trajectory1),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout),
                        cmdWithAccuracy(trajectory2),
                        new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                                Constants.AutoConstants.kAutoShooterTimeout)));

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
                currentPose.getTranslation().getDistance(finalPose.getTranslation())
                        * Constants.kMetersToInches);

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

        Command trajCmd = trajectory.cmd();

        return new FunctionalCommand(
                trajCmd::initialize,
                trajCmd::execute,
                trajCmd::end,
                () -> isFinished(trajectory, Constants.AutoConstants.kEpsilonDist),
                swerve);
    }
}
