package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    Timer timer;
    private boolean isRunning = false;
    private boolean intakeIn = true;
    private boolean jiggly = false;
    double jiggleMultiplier = 0;
    private double lastToggleTime = 0;

    //set the motors to lock mode
    public Intake(Swerve swerve) {
        this.swerve = swerve;
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        this.setDefaultCommand(setJigglyCommand(false));
        SmartDashboard.putBoolean("Intake Jiggly", false);
        timer = new Timer();
        timer.reset();
        timer.start();
        lastToggleTime = timer.get();
    
        // Don't persist parameters since it takes time and this change is temporary
        actuationMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Jiggly", jiggly);
        //custom pid instead of closed loop
        
        if (jiggly && timer.get() - lastToggleTime > 0.5){
            if (jiggleMultiplier == 0) {
                jiggleMultiplier = 1;
            } else {
                jiggleMultiplier = 0;
            }
            lastToggleTime = timer.get();
        }
        if (!jiggly) {
            jiggleMultiplier = 0;
        }

        if (intakeIn) {
            inPID.setSetpoint(Constants.IntakeConstants.kActuationOutSetpoint * Constants.IntakeConstants.inJigglyPercentage * jiggleMultiplier);
            double output = inPID.calculate(actuationMotor.getEncoder().getPosition());
            actuationMotor.set(output);            
        } else {
            outPID.setSetpoint(Constants.IntakeConstants.kActuationOutSetpoint * (1-Constants.IntakeConstants.outJigglyPercentage*jiggleMultiplier));
            double output = outPID.calculate(actuationMotor.getEncoder().getPosition());
            actuationMotor.set(output);
        }
        if (jiggly){
            driveMotor.set(Constants.IntakeConstants.kDriveSpeedOut);
        } else if (intakeIn){
            driveMotor.set(0);
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

    public void toggleJiggly() {
        jiggly = !jiggly;
    }

    public void setJiggly(boolean jiggly) {
        this.jiggly = jiggly;
    }

    private void intakeIn() {
        intakeIn = true;
        driveMotor.set(Constants.IntakeConstants.kDriveSpeedIn);
        isRunning = false;
    }

    private void intakeOut() {
        driveMotor.set(Constants.IntakeConstants.kDriveSpeedOut);
        intakeIn = false;
        isRunning = true;
    }

    public Command getIntakeOutCommand() {
        return Commands.runOnce(() -> intakeOut(), this);
    }

    public Command getIntakeInCommand() {
        return Commands.runOnce(() -> intakeIn(), this);
    }

    public Command toggleIsJigglingCommand() {
        return Commands.runOnce(() -> toggleJiggly(), this);
    }

    public Command setJigglyCommand(boolean jiggly) {
        return Commands.runOnce(() -> setJiggly(jiggly), this);
    }

    public Command toggleIntakeCommand() {
        return Commands.runOnce(() -> {
            if (intakeIn) {
                intakeOut();
            } else {
                intakeIn();
            }
        }, this);
    }
}
