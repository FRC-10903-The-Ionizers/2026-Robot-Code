package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);
    private final Swerve swerve;
    private boolean isRunning = false;

    public Shooter(Swerve swerve){
        this.swerve = swerve;
    }

    // Periodicly set the motors to the correct speed
    public void periodic() {

        if(isRunning){
            Pose2d currentPose = swerve.getPose();
            Pose2d hubPose = Constants.ShooterConstants.kHubPose;

            double distance = currentPose.getTranslation().getDistance(hubPose.getTranslation());
            double calculatedMotorSpeed = MathUtil.clamp(distance, 0, 1);

            topMotor.set(calculatedMotorSpeed);
            bottomMotor.set(calculatedMotorSpeed);
        } else {
            topMotor.set(0);
            bottomMotor.set(0);
        }
    }

    public boolean getIsRunning(){
        return isRunning;
    }

    public void setIsRunning(boolean isRunning){
        this.isRunning = isRunning;
    }

    public void toggleIsRunning(){
        isRunning = !isRunning;
    }

}
