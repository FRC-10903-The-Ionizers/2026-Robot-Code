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
    private final Swerve swerve;
    private Pose2d targetPose;
    private boolean isRunning = false;
    private final PIDController headingController = Constants.DriveConstants.kHeadingController;
    private final Controller controller;

    public Shooter(Swerve swerve, Controller controller){
        this.swerve = swerve;
        this.targetPose = new Pose2d(-1, -1, Rotation2d.fromDegrees(0));
        this.controller = controller;
    }

    // Periodicly set the motors to the correct speed
    public void periodic() {

        if(isRunning){
            topMotor.set(-0.5);
            bottomMotor.set(-0.5);

            Pose2d currentPosition = swerve.getPose();
            double currentX = currentPosition.getX();
            double currentY = currentPosition.getY();
            Translation2d translation = new Translation2d(controller.getDriveX(), controller.getDriveY());
            double target_angle = Math.atan2(this.targetPose.getY() - currentY, this.targetPose.getX() - currentX);
    
    
            headingController.setSetpoint(target_angle);
            double rotation = headingController.calculate(swerve.getPose().getRotation().getRadians());
            swerve.drive(new Translation2d(0, 0), rotation, true);
    
        } else {
            topMotor.set(0);
            bottomMotor.set(0);
        }
    }

    public boolean getIsRunning(){
        return isRunning;
    }
    
    public Pose2d getTargetPose(){
        return targetPose;
    }
    public void setTargetPose(Pose2d targetPose){
        this.targetPose = targetPose;
    }

    public void setIsRunning(boolean isRunning){
        this.isRunning = isRunning;
    }

    public void toggleIsRunning(){
        isRunning = !isRunning;
    }

}
