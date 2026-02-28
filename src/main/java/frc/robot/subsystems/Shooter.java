package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.Controller;

public class Shooter extends SubsystemBase{

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);
    private boolean isRunning = false;

    // Periodicly set the motors to the correct speed
    public void periodic() {
        if(isRunning){
            topMotor.set(-0.5);
            bottomMotor.set(-0.5);
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
