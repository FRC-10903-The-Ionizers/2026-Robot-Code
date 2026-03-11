package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.InterpolationMatrix;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Shooter extends SubsystemBase {

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    private final SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.kHoodMotorID, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final PIDController hoodPID = new PIDController(0.5, 0, 0.2);
    private double prevHoodOutput = 0; // Tune these values
    private final double kHoodFF = 0.95; // Feedforward for hood motor, tune this value
    private final double kHoodClipping = 0.3;

    private double targetSpeed = 0;
    private PIDController shooterSpeedPID = new PIDController(0.01, 0, 0.005);
    private double prevOutput = 0;

    private final Swerve swerve;
    private final InterpolationMatrix interpolationMatrix;
    private boolean isRunning = false;

    private double targetHoodAngle = 0;
    // p=0.15
    // i = 0.05
    // d = 0.07
    public Shooter(Swerve swerve, InterpolationMatrix interpolationMatrix) {
        this.swerve = swerve;
        this.interpolationMatrix = interpolationMatrix;

        hoodEncoder.setPosition(0);

        SmartDashboard.putNumber("Shooter Target Speed", 2000);
        SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Hood Target Angle", targetHoodAngle);
        SmartDashboard.putNumber("Distance to hub", 0);
    }

    // Periodicly set the motors to the correct speed
    public void periodic() {

        Optional<Alliance> ally = DriverStation.getAlliance();
        Pose2d currentPose = swerve.getPose();


        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                SmartDashboard.putNumber("Distance to Hub", (currentPose.getTranslation().getDistance(new Translation2d(11.92, 4.035))));
            }
            if (ally.get() == Alliance.Blue) {
                SmartDashboard.putNumber("Distance to Hub", (currentPose.getTranslation().getDistance(new Translation2d(4.62, 4.035))));
            }
        }
        else {
            System.out.println("Alliance not detected!");
        }

        targetHoodAngle = SmartDashboard.getNumber("Hood Target Angle", targetHoodAngle);
        hoodPID.setSetpoint(targetHoodAngle);
        double hoodOutput = hoodPID.calculate(hoodEncoder.getPosition());
        // System.out.println("Current hood angle: " + hoodEncoder.getPosition() + ", Target hood angle: " + targetHoodAngle + ", Output: " + hoodOutput + ", Current: " + hoodMotor.getOutputCurrent());
        // if (Math.abs(hoodEncoder.getVelocity()) < 0.05 && Math.abs(hoodOutput) > 0.05) {
        //     hoodOutput = Math.signum(hoodOutput) * 0.6;
        // }
        hoodOutput = MathUtil.clamp(hoodOutput, -kHoodClipping, kHoodClipping);
        hoodOutput = (prevHoodOutput * kHoodFF) + (1-kHoodFF) * hoodOutput;
        prevHoodOutput = hoodOutput;
        hoodMotor.set(-hoodOutput);
        
        // Convert angle to encoder units if needed
        // if (targetHoodAngle > hoodEncoder.getPosition()+0.05){
        //     System.out.print(hoodEncoder.getPosition());
        //     System.out.print(", target: ");
        //     System.out.print(targetHoodAngle);
        //     System.out.print(", current: ");
        //     System.out.print(hoodMotor.getOutputCurrent());
        //     System.out.println("Moving hood down");
        //     if (Math.abs(hoodEncoder.getVelocity()) < 0.01) {
        //         System.out.println("Hood is stalled, kicking motor");
        //         hoodMotor.set(-0.4);
        //     } else {
        //         hoodMotor.set(-0.1);
        //     }
        // } else if (targetHoodAngle < hoodEncoder.getPosition()-0.05){
        //     System.out.print(hoodEncoder.getPosition());
        //      System.out.print(", target: ");
        //     System.out.print(targetHoodAngle);
        //     System.out.print(", current: ");
        //     System.out.print(hoodMotor.getOutputCurrent());
        //     System.out.println("Moving hood up");
        //     if (Math.abs(hoodEncoder.getVelocity()) < 0.01) {
        //         System.out.println("Hood is stalled, kicking motor");
        //         hoodMotor.set(0.4);
        //     } else {
        //         hoodMotor.set(0.1);
        //     }
        // } else {
        //     hoodMotor.set(0);
        // }


        
        if (isRunning) {
            targetSpeed = SmartDashboard.getNumber("Shooter Target Speed", 2000);
            shooterSpeedPID.setSetpoint(-targetSpeed);
            double output = shooterSpeedPID.calculate(topMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
            output = MathUtil.clamp(output, -1, 0);
            output = (prevOutput * Constants.ShooterConstants.kShooterFF) + (1-Constants.ShooterConstants.kShooterFF) * output;
            prevOutput = output;
            topMotor.set(output);
            bottomMotor.set(-output);
            
        } else {
            topMotor.set(0);
            bottomMotor.set(0);
        }
    }

    public void setMotorSpeed(double speed) {
        this.targetSpeed = speed;
    }

    public boolean getIsRunning() {
        return isRunning;
    }

    public void setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    public void toggleIsRunning() {
        System.out.println("shooter running" + !isRunning);
        isRunning = !isRunning;
    }

    public double getHoodAngleToMove(double distanceToHub) {
        double interpolatedHoodAngle = interpolationMatrix.interpolate(distanceToHub);
        double currentHoodAngle = hoodEncoder.getPosition() * 360.0;
        return interpolatedHoodAngle - currentHoodAngle;
    }

    public void setHoodAngle(double angle) {
        this.targetHoodAngle = angle;
        System.out.println("Setting hood angle to " + angle);
        
    }
}
