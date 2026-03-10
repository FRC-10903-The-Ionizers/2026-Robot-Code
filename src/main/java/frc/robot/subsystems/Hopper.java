package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Hopper extends SubsystemBase{

    private final SparkMax spindexerMotor = new SparkMax(Constants.HopperConstants.kSpindaxerMotorID, MotorType.kBrushless);
    private final SparkMax bobMotor = new SparkMax(Constants.HopperConstants.kBobMotorID, MotorType.kBrushless);
    

    public Hopper(){
        SmartDashboard.putNumber("Spindexer Motor Speed", -0.75);
        SmartDashboard.putNumber("Bob Motor Speed", 0.75);
    }

    private boolean isSpindexerRunning = false;
    private boolean isBobRunning = false;
    // Periodicly set the motors to the correct speed
    public void periodic() {
        if(isSpindexerRunning){
            double spindexerMotorSpeed = SmartDashboard.getNumber("Spindexer Motor Speed", -0.75);
            spindexerMotor.set(spindexerMotorSpeed);
        } else {
            spindexerMotor.set(0);
        }

        if(isBobRunning){
            double bobMotorSpeed = SmartDashboard.getNumber("Bob Motor Speed", 0.75);
            bobMotor.set(bobMotorSpeed);
        } else {
            spindexerMotor.set(0);
            bobMotor.set(0);
        }
    }

    public boolean getIsSpindexerRunning(){
        return isSpindexerRunning;
    }

    public void setIsSpindexerRunning(boolean isRunning){
        this.isSpindexerRunning = isRunning;
    }

    public void toggleIsSpindexerRunning(){
        isSpindexerRunning = !isSpindexerRunning;
    }
}
