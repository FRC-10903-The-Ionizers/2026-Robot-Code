package frc.robot.util;

import static frc.robot.trajectories.ChoreoTraj.CheesyPoofsLeft$0;
import static frc.robot.trajectories.ChoreoTraj.CheesyPoofsLeft$1;
import static frc.robot.trajectories.ChoreoTraj.CheesyPoofsRight$0;
import static frc.robot.trajectories.ChoreoTraj.CheesyPoofsRight$1;
import static frc.robot.trajectories.ChoreoTraj.ChoppedCitrusRight$0;
import static frc.robot.trajectories.ChoreoTraj.ChoppedCitrusRight$1;
import static frc.robot.trajectories.ChoreoTraj.CitrusLeft$0;
import static frc.robot.trajectories.ChoreoTraj.CitrusLeft$1;
import static frc.robot.trajectories.ChoreoTraj.CitrusRight$0;
import static frc.robot.trajectories.ChoreoTraj.CitrusRight$1;
import static frc.robot.trajectories.ChoreoTraj.HearMeOut$0;
import static frc.robot.trajectories.ChoreoTraj.HearMeOut$1;
import static frc.robot.trajectories.ChoreoTraj.Left$0;
import static frc.robot.trajectories.ChoreoTraj.Left$1;
import static frc.robot.trajectories.ChoreoTraj.Outpost$0;
import static frc.robot.trajectories.ChoreoTraj.Outpost$1;
import static frc.robot.trajectories.ChoreoTraj.Right$0;
import static frc.robot.trajectories.ChoreoTraj.Right$1;
import static frc.robot.trajectories.ChoreoTraj.Depot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
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

        autoChooser.addRoutine("Chopped Citrus Right", this::choppedCitrusRight);
        autoChooser.addRoutine("Hear Me Out", this::hearMeOut);
        
        autoChooser.addRoutine("Depot", this::depot);
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

    public AutoRoutine depot(){
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory = Depot.asAutoTraj(routine);

        routine.active().onTrue(Commands.sequence(
                trajectory.resetOdometry(),
                cmdWithAccuracy(trajectory),
                new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix, 10)
        ));

        return routine;
    }

    public AutoRoutine outpost() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = Outpost$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = Outpost$1.asAutoTraj(routine);

        routine.active().onTrue(Commands.sequence(
                trajectory1.resetOdometry(),
                cmdWithAccuracy(trajectory1),
                Commands.print("First traj completed"),
                Commands.waitSeconds(5),
                Commands.print("finished waiting"),
                cmdWithAccuracy(trajectory2),
                new ShootAndDrive(swerve, shooter, intake, hopper, controller, interpolationMatrix,
                        10)));

        return routine;
    }

    public AutoRoutine rightAutoRoutine() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = Right$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = Right$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Right Auto Chosen"),
                        trajectory1.resetOdometry(),
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
                        trajectory1.resetOdometry(),
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
                        trajectory1.resetOdometry(),
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
                        trajectory1.resetOdometry(),
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
                        trajectory1.resetOdometry(),
                        Commands.print("Citrus Right Auto Chosen"),
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

    public AutoRoutine citrusLeft() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = CitrusLeft$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = CitrusLeft$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Citrus Left Auto Chosen"),
                        trajectory1.resetOdometry(),
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

    public AutoRoutine choppedCitrusRight() {
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = ChoppedCitrusRight$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = ChoppedCitrusRight$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Chopped Citrus Right Chosen"),
                        trajectory1.resetOdometry(),
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

    public AutoRoutine hearMeOut(){
        AutoRoutine routine = AutoConstants.kAutoFactory.newRoutine("Auto");
        AutoTrajectory trajectory1 = HearMeOut$0.asAutoTraj(routine);
        AutoTrajectory trajectory2 = HearMeOut$1.asAutoTraj(routine);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.print("Hear Me Out Chosen"),
                        trajectory1.resetOdometry(),
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
                () -> trajCmd.isFinished(),
                swerve);
    }
}
