package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private final SparkMax driveMotor = new SparkMax(Constants.IntakeConstants.kDriveMotorID, MotorType.kBrushless);
    private final Swerve swerve;
    private boolean isRunning = false;

    public Intake(Swerve swerve){
        this.swerve = swerve;
    }

    // Periodicly set the motor to the correct speed
    public void periodic() {
        if(isRunning){
            driveMotor.set(-0.5);
        } else {
            driveMotor.set(0);
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
