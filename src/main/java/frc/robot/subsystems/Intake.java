package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final SparkMax driveMotor = new SparkMax(Constants.IntakeConstants.kDriveMotorID, MotorType.kBrushless);
    private final SparkMax actuationMotor = new SparkMax(Constants.IntakeConstants.kActuationMotorID, MotorType.kBrushless);
    private final Swerve swerve;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;    
    private boolean isRunning = false;

    public Intake(Swerve swerve){
        this.swerve = swerve;

        closedLoopController = actuationMotor.getClosedLoopController();
        encoder = actuationMotor.getEncoder();
        motorConfig = new SparkMaxConfig();

        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(kIntakeActuationP)
        .i(kIntakeActuationI)
        .d(kIntakeActuationD)
        .outputRange(-1, 1)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);


        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    // Periodicly set the motor to the correct speed
    public void periodic() {
        if(isRunning){
            closedLoopController.setSetpoint(3, ControlType.kPosition, ClosedLoopSlot.kSlot0);
            driveMotor.set(-0.5);
        } else {
            driveMotor.set(0);
        }
    }

    public boolean getIsRunning(){
        return isRunning;
    }

    public void setIsRunning(boolean isRunning){
        this.isRunning = isRunning;
    }

    public void toggleIsRunning(){
        isRunning = !isRunning;
    }

}
