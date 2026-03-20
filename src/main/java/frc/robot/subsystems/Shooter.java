package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Shooter subsystem for the robot. Controls the flywheel motors and
 * adjustable hood for launching game pieces at variable speeds and angles.
 *
 * @since 2025-10
 */
public class Shooter extends SubsystemBase {

    // ---- Flywheel motors ----
    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    // ---- Hood motor and PID ----
    private final SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.kHoodMotorID, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final PIDController hoodPID = new PIDController(Constants.ShooterConstants.kHoodP,
            Constants.ShooterConstants.kHoodI, Constants.ShooterConstants.kHoodD);
    private double prevHoodOutput = 0;
    private boolean hoodIsZeroed = false;

    // ---- Shooter speed PID ----
    private boolean isRunning = false;
    private double targetSpeed = 0;
    private PIDController shooterSpeedPID = new PIDController(Constants.ShooterConstants.kShooterP,
            Constants.ShooterConstants.kShooterI,
            Constants.ShooterConstants.kShooterD);
    private double prevOutput = 0;

    // ---- Hood angle target ----
    private double targetHoodAngle = 0;

    /**
     * Constructs the Shooter subsystem. Sets the default command to idle
     * the shooter and initializes SmartDashboard tuning values.
     */
    public Shooter() {
        // Use the passed-in WPILib interpolating map
        this.setDefaultCommand(new InstantCommand(() -> {
            setSpeedAndAngle(Constants.ShooterConstants.kShooterIdleSpeed, 0);
            isRunning = false;
        }, this));

        hoodEncoder.setPosition(0);
        SmartDashboard.putNumber("Target Shooter Speed Offset", 0);
        SmartDashboard.putNumber("Target Hood Angle Offset", 0);
        SmartDashboard.putNumber("Hood P", Constants.ShooterConstants.kHoodP);
        SmartDashboard.putNumber("Hood I", Constants.ShooterConstants.kHoodI);
        SmartDashboard.putNumber("Hood D", Constants.ShooterConstants.kHoodD);
        SmartDashboard.putNumber("Hood LPF", Constants.ShooterConstants.kHoodLPF);

    }

    /**
     * Periodically updates hood zeroing, hood PID control, and shooter
     * flywheel speed PID. Applies low-pass filtering and battery voltage
     * compensation to motor outputs.
     */
    public void periodic() {

        // Hood zeroing sequence
        if (!hoodIsZeroed) {
            hoodMotor.set(Constants.ShooterConstants.kHoodZeroingOutput);
            if (hoodMotor.getOutputCurrent() >= Constants.ShooterConstants.kHoodStallCurrentAmps) {
                hoodMotor.set(0);
                hoodIsZeroed = true;
                hoodEncoder.setPosition(0);
            }
        } else {
            // Hood PID with live-tunable gains
            hoodPID.setP(SmartDashboard.getNumber("Hood P", Constants.ShooterConstants.kHoodP));
            hoodPID.setI(SmartDashboard.getNumber("Hood I", Constants.ShooterConstants.kHoodI));
            hoodPID.setD(SmartDashboard.getNumber("Hood D", Constants.ShooterConstants.kHoodD));
            double kHoodLPF = SmartDashboard.getNumber("Hood LPF", Constants.ShooterConstants.kHoodLPF);
            hoodPID.setSetpoint(MathUtil.clamp(targetHoodAngle, 0, Constants.ShooterConstants.kHoodMaxAngleRad));
            double hoodOutput = hoodPID.calculate(hoodEncoder.getPosition());

            hoodOutput = MathUtil.clamp(hoodOutput, -Constants.ShooterConstants.kHoodClipping, Constants.ShooterConstants.kHoodClipping);
            hoodOutput = (prevHoodOutput * kHoodLPF) + (1 - kHoodLPF) * hoodOutput;
            prevHoodOutput = hoodOutput;
            hoodMotor.set(-hoodOutput);
        }

        // Shooter flywheel speed PID with low-pass filter and voltage compensation
        shooterSpeedPID.setSetpoint(-targetSpeed);
        double output = shooterSpeedPID.calculate(topMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
        output = MathUtil.clamp(output, -1, 0);
        SmartDashboard.putNumber("Shooter Speed Error", shooterSpeedPID.getPositionError());
        SmartDashboard.putNumber("Hood Error", hoodPID.getPositionError());
        output = ((prevOutput * Constants.ShooterConstants.kShooterLPF) + (1 - Constants.ShooterConstants.kShooterLPF) *
                (output - (targetSpeed * Constants.ShooterConstants.kShooterKV)));
        prevOutput = output;
        output *= (Constants.ShooterConstants.kNominalBatteryVoltage / RobotController.getBatteryVoltage());
        topMotor.set(output);
        bottomMotor.set(-output);

    }

    /**
     * Sets the target flywheel speed.
     *
     * @param speed the target speed in RPM
     */
    public void setMotorSpeed(double speed) {
        this.targetSpeed = speed;
    }

    /**
     * Sets both the target flywheel speed and hood angle, applying
     * SmartDashboard offsets for live tuning.
     *
     * @param speed the target flywheel speed in RPM
     * @param angle the target hood angle in radians
     */
    public void setSpeedAndAngle(double speed, double angle) {
        this.targetSpeed = speed + SmartDashboard.getNumber("Target Shooter Speed Offset", 0);
        shooterSpeedPID.setSetpoint(-this.targetSpeed);
        this.targetHoodAngle = angle + SmartDashboard.getNumber("Target Hood Angle Offset", 0);
    }

    /**
     * Returns whether the shooter is ready to shoot. The shooter is considered
     * ready once the flywheel speed error falls within tolerance.
     *
     * @return true if the shooter speed is within tolerance
     */
    public boolean getIsReadyToShoot() {
        if (Math.abs(shooterSpeedPID.getError()) < Constants.ShooterConstants.kShooterSpeedToleranceRPM) {
            isRunning = true;
        }
        return isRunning;
    }

    /**
     * Sets the target hood angle directly.
     *
     * @param angle the target hood angle in radians
     */
    public void setHoodAngle(double angle) {
        this.targetHoodAngle = angle;
        System.out.println("Setting hood angle to " + angle);

    }
}
