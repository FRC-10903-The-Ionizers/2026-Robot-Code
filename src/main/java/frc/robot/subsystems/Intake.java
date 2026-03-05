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
    private boolean intakeIn = false;
    public Intake(Swerve swerve) {
        this.swerve = swerve;
        config = new SparkMaxConfig();

    }
    @Override
    public void periodic() {
        //custom pid instead of closed loop
        if (intakeIn) {
            inPID.setSetpoint(0);
            double output = inPID.calculate(actuationMotor.getEncoder().getPosition());
            actuationMotor.set(output);
        } else {
            outPID.setSetpoint(4.6);
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
        driveMotor.set(0);
        intakeIn = true;
    }

    private void intakeOut() {
        intakeIn = false;
        driveMotor.set(0.5);
    }

    public Command getIntakeOutCommand() {
        return Commands.runOnce(() -> intakeOut(), this);
    }

    public Command getIntakeInCommand() {
        return Commands.runOnce(() -> intakeIn(), this);
    }
}
