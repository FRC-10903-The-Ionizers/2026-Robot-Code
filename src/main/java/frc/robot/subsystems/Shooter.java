package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.InterpolationMatrix;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    private final SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.kHoodMotorID, MotorType.kBrushless);
    private final SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    private final Swerve swerve;
    private final InterpolationMatrix interpolationMatrix;
    private boolean isRunning = false;

    public Shooter(Swerve swerve, InterpolationMatrix interpolationMatrix) {
        this.swerve = swerve;
        this.interpolationMatrix = interpolationMatrix;

        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .i(0)
                .d(0)
                .outputRange(-1, 1);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodEncoder.setPosition(0);
    }

    // Periodicly set the motors to the correct speed
    public void periodic() {

        if (isRunning) {
            Pose2d currentPose = swerve.getPose();
            Pose2d hubPose = Constants.ShooterConstants.kHubPose;

            double distance = currentPose.getTranslation().getDistance(hubPose.getTranslation());
            double changeInHoodAngle = getHoodAngleToMove(distance);

            double targetRotations = changeInHoodAngle / 360.0;
            hoodController.setSetpoint(targetRotations, ControlType.kPosition);

            topMotor.set(-0.5);
            bottomMotor.set(-0.5);
        } else {
            topMotor.set(0);
            bottomMotor.set(0);
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

    public double getHoodAngleToMove(double distanceToHub) {
        double interpolatedHoodAngle = interpolationMatrix.interpolate(distanceToHub);
        double currentHoodAngle = hoodEncoder.getPosition() * 360.0;
        return interpolatedHoodAngle - currentHoodAngle;
    }
}
