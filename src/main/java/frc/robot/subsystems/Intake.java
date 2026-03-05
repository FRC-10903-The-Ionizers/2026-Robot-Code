package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax driveMotor = new SparkMax(Constants.IntakeConstants.kDriveMotorID, MotorType.kBrushless);
    //private final SparkMax actuationMotor = new SparkMax(Constants.IntakeConstants.kActuationMotorID, MotorType.kBrushless);
    private final Swerve swerve;
    public final PIDController inPID = new PIDController(0.3, 0, 0);
    public final PIDController outPID = new PIDController(0.08, 0, 0);
    private SparkClosedLoopController closedLoopController;
    private final SparkMax actuationMotor = new SparkMax(Constants.IntakeConstants.kActuationMotorID, MotorType.kBrushless);
    private SparkMaxConfig config;
    private RelativeEncoder encoder;
    private boolean isRunning = false;
    private double init_position = 0;
    private double target_position = 0;
    public Intake(Swerve swerve) {
        this.swerve = swerve;
        config = new SparkMaxConfig();
        config.closedLoop.pid(0.08, 0, 0);
        config.softLimit.reverseSoftLimit(0.0);
        config.softLimit.reverseSoftLimitEnabled(false); // Temporarily disable to test
        config.softLimit.forwardSoftLimit(5.0);
        config.softLimit.forwardSoftLimitEnabled(false);

        init_position = actuationMotor.getEncoder().getPosition();
        actuationMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    }

    @Override
    public void periodic() {
        
        if (isRunning) {
            // SparkClosedLoopController closedLoopController = actuationMotor.getClosedLoopController();
            // //log the encoder position
            // closedLoopController.setSetpoint(4.8, ControlType.kPosition); // 10 RPM
        } else {
        }
        System.out.println("Intake position: " + actuationMotor.getEncoder().getPosition());
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

    private void intakeIn() {
        driveMotor.set(0);
        config.closedLoop.pid(0.08, 0, 0);
        SparkClosedLoopController closedLoopController = actuationMotor.getClosedLoopController();
        closedLoopController.setSetpoint(0, ControlType.kPosition);
    }

    private void intakeOut() {
        config.closedLoop.pid(0.08, 0, 0);
        SparkClosedLoopController closedLoopController = actuationMotor.getClosedLoopController();
        closedLoopController.setSetpoint(4.6, ControlType.kPosition);
        driveMotor.set(0.5);
    }

    public Command getIntakeOutCommand() {
        return Commands.runOnce(() -> intakeOut(), this);
    }

    public Command getIntakeInCommand() {
        return Commands.runOnce(() -> intakeIn(), this);
    }
}
