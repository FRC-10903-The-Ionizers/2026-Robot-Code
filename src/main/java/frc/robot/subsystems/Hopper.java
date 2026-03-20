package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Hopper subsystem for the robot. Controls the spindexer and bob motors
 * that feed game pieces from the intake into the shooter.
 *
 * @since 2025-10
 */
public class Hopper extends SubsystemBase {

    // ---- Motors ----
    private final SparkMax spindexerMotor = new SparkMax(Constants.HopperConstants.kSpindexerMotorID, MotorType.kBrushless);
    private final SparkMax bobMotor = new SparkMax(Constants.HopperConstants.kBobMotorID, MotorType.kBrushless);

    // ---- State ----
    private boolean isSpindexerRunning = false;

    /**
     * Constructs the Hopper subsystem. Sets the default command to stop
     * the hopper and publishes tuning values to SmartDashboard.
     */
    public Hopper() {
        this.setDefaultCommand(new InstantCommand(() -> setIsHopperRunning(false), this));
        SmartDashboard.putNumber("Hopper Spindexer Speed", Constants.HopperConstants.kSpindexerSpeed);
        SmartDashboard.putNumber("Hopper Bob Speed", Constants.HopperConstants.kBobSpeed);
    }

    /**
     * Periodically sets the motors to the correct speed based on
     * the current running state. Reads speed values from SmartDashboard
     * to allow live tuning.
     */
    public void periodic() {
        if (isSpindexerRunning) {
            spindexerMotor.set(SmartDashboard.getNumber("Hopper Spindexer Speed", Constants.HopperConstants.kSpindexerSpeed));
            bobMotor.set(SmartDashboard.getNumber("Hopper Bob Speed", Constants.HopperConstants.kBobSpeed));
        } else {
            spindexerMotor.set(0);
            bobMotor.set(0);
        }
    }

    /**
     * Returns whether the spindexer is currently running.
     *
     * @return true if the spindexer is running
     */
    public boolean getIsSpindexerRunning() {
        return isSpindexerRunning;
    }

    /**
     * Sets the hopper running state.
     *
     * @param isRunning true to run the hopper, false to stop
     */
    public void setIsHopperRunning(boolean isRunning) {
        this.isSpindexerRunning = isRunning;
    }

    /**
     * Toggles the spindexer between running and stopped.
     */
    public void toggleIsSpindexerRunning() {
        isSpindexerRunning = !isSpindexerRunning;
    }
}
