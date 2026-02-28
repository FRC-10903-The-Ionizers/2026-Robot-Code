package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax driveMotor = new SparkMax(Constants.IntakeConstants.kDriveMotorID, MotorType.kBrushless);
    private final SparkMax actuationMotor = new SparkMax(Constants.IntakeConstants.kActuationMotorID, MotorType.kBrushless);
    private final Swerve swerve;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private boolean isRunning = false;

    public Intake(Swerve swerve) {
        this.swerve = swerve;

        closedLoopController = actuationMotor.getClosedLoopController();
        encoder = actuationMotor.getEncoder();
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.IntakeConstants.kIntakeActuationP)
            .i(Constants.IntakeConstants.kIntakeActuationI)
            .d(Constants.IntakeConstants.kIntakeActuationD)
            .outputRange(-1, 1)
            .p(0.0001, ClosedLoopSlot.kSlot1)
            .i(0, ClosedLoopSlot.kSlot1)
            .d(0, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
            .velocityFF(12.0 / 5767, ClosedLoopSlot.kSlot1);

        actuationMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        if (isRunning) {
            closedLoopController.setReference(3, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            driveMotor.set(-0.5);
        } else {
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
}
