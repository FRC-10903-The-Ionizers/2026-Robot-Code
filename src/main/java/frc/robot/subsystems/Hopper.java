package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Hopper extends SubsystemBase{

    private final SparkMax spindexerMotor = new SparkMax(Constants.HopperConstants.kSpindaxerMotorID, MotorType.kBrushless);
    private final SparkMax bobMotor = new SparkMax(Constants.HopperConstants.kBobMotorID, MotorType.kBrushless);
    

    public Hopper(){
        this.setDefaultCommand(new InstantCommand(() -> setIsHopperRunning(false), this));
    }

    private boolean isSpindexerRunning = false;
    // Periodicly set the motors to the correct speed
    public void periodic() {
        if(isSpindexerRunning){
            spindexerMotor.set(Constants.HopperConstants.kSpindaxerSpeed);
            bobMotor.set(Constants.HopperConstants.kBobSpeed);
        } else {
            spindexerMotor.set(0);
            bobMotor.set(0);
        }
    }

    public boolean getIsSpindexerRunning(){
        return isSpindexerRunning;
    }

    public void setIsHopperRunning(boolean isRunning){
        this.isSpindexerRunning = isRunning;
    }

    public void toggleIsSpindexerRunning(){
        isSpindexerRunning = !isSpindexerRunning;
    }
}
