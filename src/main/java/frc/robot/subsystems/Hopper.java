package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.time.Instant;
import frc.robot.Constants;

/**
 * Hopper subsystem for the robot. Controls the spindexer and bob motors that feed game pieces from
 * the intake into the shooter.
 *
 * @since 2025-10
 */
public class Hopper extends SubsystemBase {

  // ---- Motors ----
  private final SparkMax spindexerMotor =
      new SparkMax(Constants.HopperConstants.kSpindexerMotorID, MotorType.kBrushless);
  private final SparkMax bobMotor =
      new SparkMax(Constants.HopperConstants.kBobMotorID, MotorType.kBrushless);

  // ---- State ----
  private boolean isSpindexerRunning = false;
  private int isSpindexerBackwards = -1;

  private Instant spindexerStuckStartTime = Instant.MIN;

  /**
   * Constructs the Hopper subsystem. Sets the default command to stop the hopper and publishes
   * tuning values to SmartDashboard.
   */
  public Hopper() {
    this.setDefaultCommand(new InstantCommand(() -> setIsHopperRunning(false), this));
    SmartDashboard.putNumber("Hopper Spindexer Speed", Constants.HopperConstants.kSpindexerSpeed);
    SmartDashboard.putNumber("Hopper Bob Speed", Constants.HopperConstants.kBobSpeed);
  }

  /**
   * Periodically sets the motors to the correct speed based on the current running state. Reads
   * speed values from SmartDashboard to allow live tuning.
   */
  public void periodic() {
    if (isSpindexerRunning && Instant.now().isAfter(spindexerStuckStartTime.plusMillis(Constants.HopperConstants.kSpindexerStuckUnstuckTime))) {

      // detect whether spindexer is stuck with voltage

      Boolean isSpindexerStuck = spindexerMotor.getOutputCurrent() > Constants.HopperConstants.kSpindexerStuckCurrentThreshold && spindexerMotor.getEncoder().getVelocity() < Constants.HopperConstants.kSpindexerStuckVelocityThreshold;

      if (isSpindexerStuck) {
        // If the spindexer is stuck, reverse it for .1 seconds to try to clear the jam
        spindexerStuckStartTime = Instant.now();
      }
        spindexerMotor.set(
            isSpindexerBackwards*SmartDashboard.getNumber(
                "Hopper Spindexer Speed", Constants.HopperConstants.kSpindexerSpeed));
        bobMotor.set(
            SmartDashboard.getNumber("Hopper Bob Speed", Constants.HopperConstants.kBobSpeed));
    }
    else if (isSpindexerRunning && Instant.now().isBefore(spindexerStuckStartTime.plusMillis(Constants.HopperConstants.kSpindexerStuckUnstuckTime))) {
      // If the spindexer is currently in the "unstucking" phase, run it backwards
      spindexerMotor.set(
          -isSpindexerBackwards*SmartDashboard.getNumber(
              "Hopper Spindexer Speed", Constants.HopperConstants.kSpindexerSpeed));
      bobMotor.set(
          -SmartDashboard.getNumber("Hopper Bob Speed", Constants.HopperConstants.kBobSpeed));

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

  /** Toggles the spindexer between running and stopped. */
  public void toggleIsSpindexerRunning() {
    isSpindexerRunning = !isSpindexerRunning;
  }
  
  public void setSpindexerBackwards(boolean isBackwards){
    if (isBackwards) {
      isSpindexerBackwards = -1;
    } else {
      isSpindexerBackwards = 1;
    }
  }
}
