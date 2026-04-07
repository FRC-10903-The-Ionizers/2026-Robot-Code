package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Intake subsystem for the robot. Controls the drive motor and actuation motor
 * for
 * deploying/retracting the intake, and supports a "jiggle" mode for dislodging
 * stuck game pieces.
 *
 * @since 2025-10
 */
public class Intake extends SubsystemBase {

    // ---- Motors and PID ----
    private final SparkMax driveMotor = new SparkMax(Constants.IntakeConstants.kDriveMotorID, MotorType.kBrushless);
    // private final SparkMax actuationMotor = new
    // SparkMax(Constants.IntakeConstants.kActuationMotorID, MotorType.kBrushless);
    private final Swerve swerve;
    public final PIDController inPid = new PIDController(
            Constants.IntakeConstants.kInP,
            Constants.IntakeConstants.kInI,
            Constants.IntakeConstants.kInD);
    public final PIDController outPid = new PIDController(
            Constants.IntakeConstants.kOutP,
            Constants.IntakeConstants.kOutI,
            Constants.IntakeConstants.kOutD);
    private final SparkMax actuationMotor = new SparkMax(Constants.IntakeConstants.kActuationMotorID,
            MotorType.kBrushless);

    private final Timer timer;

    // ---- State ----
    private boolean isRunning = false;
    private boolean intakeIn = true;
    private boolean jiggly = false;
    private boolean jigglyEnabled = true;
    private double jiggleMultiplier = 0;
    private double lastToggleTime = 0;

    /**
     * Constructs the Intake subsystem. Configures the actuation motor for brake
     * mode, starts the
     * jiggle timer, and sets the default command.
     *
     * @param swerve the swerve subsystem (used for pose-aware intake logic)
     */
    public Intake(Swerve swerve) {
        this.swerve = swerve;
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        this.setDefaultCommand(setJigglyCommand(false));
        SmartDashboard.putBoolean("Intake Jiggly", false);
        timer = new Timer();
        timer.reset();
        timer.start();
        lastToggleTime = timer.get();
        SmartDashboard.putNumber("Intake Speed", Constants.IntakeConstants.kDriveSpeedOut);

        // Don't persist parameters since it takes time and this change is temporary
        actuationMotor.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /**
     * Periodically updates the intake actuation PID and drive motor output. Handles
     * jiggle mode
     * toggling based on a configurable time interval.
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Jiggly", jiggly);
        // custom pid instead of closed loop

        // Jiggle mode toggling
        if (jiggly && jigglyEnabled
                && timer.get() - lastToggleTime > Constants.IntakeConstants.kJiggleIntervalSeconds) {
            if (jiggleMultiplier == 0) {
                jiggleMultiplier = 1;
            } else {
                jiggleMultiplier = 0;
            }
            lastToggleTime = timer.get();
        }
        if (!jiggly || !jigglyEnabled) {
            jiggleMultiplier = 0;
        }

        // Actuation PID control
        if (intakeIn) {
            inPid.setSetpoint(
                    Constants.IntakeConstants.kActuationOutSetpoint
                            * Constants.IntakeConstants.kInJigglyPercentage
                            * jiggleMultiplier);
            double output = inPid.calculate(actuationMotor.getEncoder().getPosition());
            actuationMotor.set(output);
        } else {
            outPid.setSetpoint(
                    Constants.IntakeConstants.kActuationOutSetpoint
                            * (1 - Constants.IntakeConstants.kOutJigglyPercentage * jiggleMultiplier));
            double output = outPid.calculate(actuationMotor.getEncoder().getPosition());
            actuationMotor.set(output);
        }

        // Drive motor control
        if (jiggly && jigglyEnabled) {
            driveMotor.set(SmartDashboard.getNumber("Intake Speed", Constants.IntakeConstants.kDriveSpeedOut));
        } else if (intakeIn) {
            driveMotor.set(0);
        }
    }

    // ---- State getters and setters ----

    /**
     * Returns whether the intake is currently running.
     *
     * @return true if the intake is running
     */
    public boolean getIsRunning() {
        return isRunning;
    }

    /**
     * Sets the intake running state.
     *
     * @param isRunning true to run the intake, false to stop
     */
    public void setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    /** Toggles the intake between running and stopped. */
    public void toggleIsRunning() {
        isRunning = !isRunning;
    }

    /** Toggles jiggle mode on or off. */
    public void toggleJiggly() {

        jiggly = !jiggly;
    }

    /**
     * Sets the jiggle mode state.
     *
     * @param jiggly true to enable jiggle mode, false to disable
     */
    public void setJiggly(boolean jiggly) {
        this.jiggly = jiggly;
    }

    // ---- Actuation methods ----

    /** Retracts the intake inward and starts the drive motor at intake speed. */
    private void intakeIn() {
        intakeIn = true;
        driveMotor.set(Constants.IntakeConstants.kDriveSpeedIn);
        isRunning = false;
    }

    /** Deploys the intake outward and starts the drive motor at outtake speed. */
    private void intakeOut() {
        driveMotor.set(SmartDashboard.getNumber("Intake Speed", Constants.IntakeConstants.kDriveSpeedOut));
        intakeIn = false;
        isRunning = true;
    }

    // ---- Command factories ----

    /**
     * Returns a command that deploys the intake outward.
     *
     * @return an {@link Command} that runs {@link #intakeOut()}
     */
    public Command getIntakeOutCommand() {
        return Commands.runOnce(() -> intakeOut(), this);
    }

    /**
     * Returns a command that retracts the intake inward.
     *
     * @return an {@link Command} that runs {@link #intakeIn()}
     */
    public Command getIntakeInCommand() {
        return Commands.runOnce(() -> intakeIn(), this);
    }

    /**
     * Returns a command that toggles jiggle mode.
     *
     * @return an {@link Command} that runs {@link #toggleJiggly()}
     */
    public Command toggleIsJigglingCommand() {
        return Commands.runOnce(() -> toggleJiggly());
    }

    public void toggleJigglyEnabled() {
        jigglyEnabled = !jigglyEnabled;
    }

    /**
     * Returns a command that sets jiggle mode to the specified state.
     *
     * @param jiggly true to enable jiggle mode, false to disable
     * @return an {@link Command} that runs {@link #setJiggly(boolean)}
     */
    public Command setJigglyCommand(boolean jiggly) {
        return Commands.runOnce(() -> setJiggly(jiggly), this);
    }

    /**
     * Returns a command that toggles the intake between deployed and retracted.
     *
     * @return an {@link Command} that toggles intake direction
     */
    public Command toggleIntakeCommand() {
        return Commands.runOnce(
                () -> {
                    if (intakeIn) {
                        intakeOut();
                    } else {
                        intakeIn();
                    }
                });
    }
}
