package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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
    public final PIDController inPID = new PIDController(
            Constants.IntakeConstants.kInP, Constants.IntakeConstants.kInI, Constants.IntakeConstants.kInD);
    public final PIDController outPID = new PIDController(
            Constants.IntakeConstants.kOutP, Constants.IntakeConstants.kOutI, Constants.IntakeConstants.kOutD);
    private final SparkMax actuationMotor = new SparkMax(Constants.IntakeConstants.kActuationMotorID, MotorType.kBrushless);
    private boolean isRunning = false;
    private boolean intakeIn = false;

    //set the motors to lock mode
    public Intake(Swerve swerve) {
        this.swerve = swerve;
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
    
        // Don't persist parameters since it takes time and this change is temporary
        actuationMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    @Override
    public void periodic() {
        //custom pid instead of closed loop
        if (intakeIn) {
            inPID.setSetpoint(0);
            double output = inPID.calculate(actuationMotor.getEncoder().getPosition());
            actuationMotor.set(output);
        } else {
            outPID.setSetpoint(Constants.IntakeConstants.kActuationOutSetpoint);
            double output = outPID.calculate(actuationMotor.getEncoder().getPosition());

            actuationMotor.set(output);
        }
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
        intakeIn = true;
        driveMotor.set(Constants.IntakeConstants.kDriveSpeedIn);
    }

    private void intakeOut() {
        driveMotor.set(Constants.IntakeConstants.kDriveSpeedOut);
        intakeIn = false;
    }

    public Command getIntakeOutCommand() {
        return Commands.runOnce(() -> intakeOut(), this);
    }

    public Command getIntakeInCommand() {
        return Commands.runOnce(() -> intakeIn(), this);
    }
}
