package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.InterpolationMatrix;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    private final SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.kHoodMotorID, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final PIDController hoodPID = new PIDController(0.2, 0, 0); // Tune these values

    private double targetSpeed = 0;
    private PIDController shooterSpeedPID = new PIDController(0.01, 0, 0.005);
    private double prevOutput = 0;

    private final Swerve swerve;
    private final InterpolationMatrix interpolationMatrix;
    private boolean isRunning = false;

    private double targetHoodAngle = 0;

    public Shooter(Swerve swerve, InterpolationMatrix interpolationMatrix) {
        this.swerve = swerve;
        this.interpolationMatrix = interpolationMatrix;

        hoodEncoder.setPosition(0);

        SmartDashboard.putNumber("Shooter Target Speed", 2000);
        SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
    }

    // Periodicly set the motors to the correct speed
    public void periodic() {
        // Convert angle to encoder units if needed
        if (targetHoodAngle > hoodEncoder.getPosition()+0.05){
            hoodMotor.set(-0.1);
        } else if (targetHoodAngle < hoodEncoder.getPosition()-0.05){
            hoodMotor.set(0.1);
        } else {
            hoodMotor.set(0);
        }


        System.out.println(hoodMotor.getEncoder().getPosition());
        
        if (isRunning) {
            Pose2d currentPose = swerve.getPose();
            Pose2d hubPose = Constants.ShooterConstants.kHubPose;

            double distance = currentPose.getTranslation().getDistance(hubPose.getTranslation());

            targetSpeed = SmartDashboard.getNumber("Shooter Target Speed", 2000);
            shooterSpeedPID.setSetpoint(-targetSpeed);
            double output = shooterSpeedPID.calculate(topMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
            output = MathUtil.clamp(output, 0, 1);
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
