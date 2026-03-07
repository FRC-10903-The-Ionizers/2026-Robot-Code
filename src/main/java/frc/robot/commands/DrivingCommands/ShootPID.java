package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

public class ShootPID extends Command {
    /**
     * DriveWhileLocked command for the robot. Drives the robot while maintaining a locked orientation.
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private double speed;

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    public ShootPID(double speed) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve: the swerve drive
         * @param controller: the PID controller
         */
        this.speed = speed;

        
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.001)
                .i(0)
                .d(0)
                .outputRange(-1, 1);

        topMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SparkClosedLoopController topClosedLoopController = topMotor.getClosedLoopController();
        SparkClosedLoopController bottomClosedLoopController = bottomMotor.getClosedLoopController();
        topClosedLoopController.setSetpoint(-speed, ControlType.kVelocity);
        bottomClosedLoopController.setSetpoint(speed, ControlType.kVelocity);
        System.out.println("Motors configured");
    }

    

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */

        System.out.println("Set shooter speeds: " + topMotor.getEncoder().getVelocity() + " (top), " + bottomMotor.getEncoder().getVelocity() + " (bottom)");
    }

    @Override
    public void end(boolean interrupted) {
        SparkClosedLoopController topClosedLoopController = topMotor.getClosedLoopController();
        SparkClosedLoopController bottomClosedLoopController = bottomMotor.getClosedLoopController();
        topClosedLoopController.setSetpoint(0, ControlType.kVelocity);
        bottomClosedLoopController.setSetpoint(0, ControlType.kVelocity);
    }
}