package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase{

    private final SparkMax spindexerMotor = new SparkMax(Constants.HopperConstants.kSpindaxerMotorID, MotorType.kBrushless);
    private final SparkMax bobMotor = new SparkMax(Constants.HopperConstants.kBobMotorID, MotorType.kBrushless);

    private boolean isspindexerRunning = false;
    private boolean isBobRunning = false;
    // Periodicly set the motors to the correct speed
    public void periodic() {
        System.out.println("hopper periodic, isspindexerRunning: " + isspindexerRunning + ", isBobRunning: " + isBobRunning);
        if(isspindexerRunning){
            spindexerMotor.set(-0.75);
        } else {
            spindexerMotor.set(0);
        }

        if(isBobRunning){
            bobMotor.set(0.75);
        } else {
            bobMotor.set(0);
        }
    }

    public boolean getIsSpindexerRunning(){
        return isspindexerRunning;
    }

    public boolean getIsBobRunning(){
        return isBobRunning;
    }

    public void setIsSpindexerRunning(boolean isRunning){
        this.isspindexerRunning = isRunning;
    }

    public void setIsBobRunning(boolean isRunning){
        this.isBobRunning = isRunning;
    }

    public void toggleIsSpindexerRunning(){
        isspindexerRunning = !isspindexerRunning;
    }

    public void toggleIsBobRunning(){
        isBobRunning = !isBobRunning;
    }

}
