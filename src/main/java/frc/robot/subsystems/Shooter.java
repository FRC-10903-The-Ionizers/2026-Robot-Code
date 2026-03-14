package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class Shooter extends SubsystemBase {

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    private final SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.kHoodMotorID, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final PIDController hoodPID = new PIDController(Constants.ShooterConstants.kHoodP, Constants.ShooterConstants.kHoodI, Constants.ShooterConstants.kHoodD);
    private double prevHoodOutput = 0; // Tune these values
    private double kHoodLPF = 0.95; // Feedforward for hood motor, tune this value
    private final double kHoodClipping = 0.3;
    private boolean hoodIsZeroed = false;
    private boolean isRunning = false;

    private double targetSpeed = 0;
    private PIDController shooterSpeedPID = new PIDController(Constants.ShooterConstants.kShooterP, 
                                                                Constants.ShooterConstants.kShooterI, 
                                                                Constants.ShooterConstants.kShooterD);
    private double prevOutput = 0;

    private double targetHoodAngle = 0;

    public Shooter() {
        // Use the passed-in WPILib interpolating map
        this.setDefaultCommand(new InstantCommand(() -> {setSpeedAndAngle(2000, 0); isRunning=false;}, this));

        hoodEncoder.setPosition(0);

    }

    // Periodicly set the motors to the correct speed
    public void periodic() {

        if (!hoodIsZeroed){
            hoodMotor.set(0.2);
            if (hoodMotor.getOutputCurrent() >= 45) { // If the motor is stalled, we can assume it's at the zero position
                hoodMotor.set(0);
                hoodIsZeroed = true;
                hoodEncoder.setPosition(0);
            }
        } else {
            //clamp target hood angle to prevent damage to the motor, limit it at 1.4 rad
            hoodPID.setSetpoint(MathUtil.clamp(targetHoodAngle, 0, 1.4));
            double hoodOutput = hoodPID.calculate(hoodEncoder.getPosition());

            hoodOutput = MathUtil.clamp(hoodOutput, -kHoodClipping, kHoodClipping);
            hoodOutput = (prevHoodOutput * kHoodLPF) + (1-kHoodLPF) * hoodOutput;
            prevHoodOutput = hoodOutput;
            hoodMotor.set(-hoodOutput);
        }

                
        shooterSpeedPID.setSetpoint(-targetSpeed);
        double output = shooterSpeedPID.calculate(topMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
        output = MathUtil.clamp(output, -1, 0);
        SmartDashboard.putNumber("Shooter Speed Error", shooterSpeedPID.getPositionError());
        output = ((prevOutput * Constants.ShooterConstants.kShooterLPF) + (1-Constants.ShooterConstants.kShooterLPF) * 
                    (output - (targetSpeed * Constants.ShooterConstants.kShooterKV)));
        prevOutput = output;
        output *= (12.1/RobotController.getBatteryVoltage());
        topMotor.set(output);
        bottomMotor.set(-output);
            

    }

    public void setMotorSpeed(double speed) {
        this.targetSpeed = speed;
    }

    public void setSpeedAndAngle(double speed, double angle) {
        this.targetSpeed = speed;
        shooterSpeedPID.setSetpoint(-speed);
        this.targetHoodAngle = angle;
    }

    public boolean getIsReadyToShoot() {
        if (Math.abs(shooterSpeedPID.getError()) < 25) {
            isRunning = true;
        }
        return isRunning;
    }

    public void setHoodAngle(double angle) {
        this.targetHoodAngle = angle;
        System.out.println("Setting hood angle to " + angle);
        
    }
}
