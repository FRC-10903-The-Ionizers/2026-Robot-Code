package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class Shooter extends SubsystemBase {

    private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
    private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);

    private final SparkMax hoodMotor = new SparkMax(Constants.ShooterConstants.kHoodMotorID, MotorType.kBrushless);
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final PIDController hoodPID = new PIDController(0.7, 0, 0.4);
    private double prevHoodOutput = 0; // Tune these values
    private final double kHoodFF = 0.95; // Feedforward for hood motor, tune this value
    private final double kHoodClipping = 0.3;

    private double targetSpeed = 0;
    private PIDController shooterSpeedPID = new PIDController(0.01, 0, 0.004);
    private double prevOutput = 0;

    private final Swerve swerve;
    private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix;
    private boolean isRunning = false;
    private boolean isFullSpeed = false;

    private double targetHoodAngle = 0;
    // p=0.15
    // i = 0.05
    // d = 0.07
    public Shooter(Swerve swerve, InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix) {
        this.swerve = swerve;
        // Use the passed-in WPILib interpolating map
        this.interpolationMatrix = interpolationMatrix;

        hoodEncoder.setPosition(0);

        SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Hood Target Angle", targetHoodAngle);
        SmartDashboard.putNumber("Distance to hub", 0);
        SmartDashboard.putNumber("Shooter Target Speed", 0);
    }

    // Periodicly set the motors to the correct speed
    public void periodic() {
        
        Pose2d currentPose = swerve.getPose();
        double distanceToHub = 0;

        Translation2d targetHubTranslation = RobotContainer.getTargetHubTranslation();
        SmartDashboard.putNumber("Distance to Hub", (currentPose.getTranslation().getDistance(targetHubTranslation)));
        distanceToHub = currentPose.getTranslation().getDistance(targetHubTranslation);


        // Query the interpolating tree map. The matrix stores [shooterSpeed; hoodAngle]
        Matrix<N2, N1> interp = interpolationMatrix.get(distanceToHub);
        double interpolatedShooterSpeed = 0.0;
        double interpolatedHoodAngle = 0.0;
        if (interp != null) {
            interpolatedShooterSpeed = interp.get(0, 0);
            interpolatedHoodAngle = interp.get(1, 0);
        }

        targetHoodAngle = SmartDashboard.getNumber("Hood Target Angle", targetHoodAngle);
        //clamp target hood angle to prevent damage to the motor, limit it at 1.4 rad
        hoodPID.setSetpoint(MathUtil.clamp(interpolatedHoodAngle, 0, 1.4));
        double hoodOutput = hoodPID.calculate(hoodEncoder.getPosition());
        // hoodMotor.set(hoodOutput);

        // System.out.println("Current hood angle: " + hoodEncoder.getPosition() + ", Target hood angle: " + targetHoodAngle + ", Output: " + hoodOutput + ", Current: " + hoodMotor.getOutputCurrent());
        // if (Math.abs(hoodEncoder.getVelocity()) < 0.05 && Math.abs(hoodOutput) > 0.05) {
        //     hoodOutput = Math.signum(hoodOutput) * 0.6;
        // }
        hoodOutput = MathUtil.clamp(hoodOutput, -kHoodClipping, kHoodClipping);
                hoodOutput = (prevHoodOutput * kHoodFF) + (1-kHoodFF) * hoodOutput;
        prevHoodOutput = hoodOutput;
hoodMotor.set(-hoodOutput);
        
        // Convert angle to encoder units if needed
        // if (targetHoodAngle > hoodEncoder.getPosition()+0.05){
        //     System.out.print(hoodEncoder.getPosition());
        //     System.out.print(", target: ");
        //     System.out.print(targetHoodAngle);
        //     System.out.print(", current: ");
        //     System.out.print(hoodMotor.getOutputCurrent());
        //     System.out.println("Moving hood down");
        //     if (Math.abs(hoodEncoder.getVelocity()) < 0.01) {
        //         System.out.println("Hood is stalled, kicking motor");
        //         hoodMotor.set(-0.4);
        //     } else {
        //         hoodMotor.set(-0.1);
        //     }
        // } else if (targetHoodAngle < hoodEncoder.getPosition()-0.05){
        //     System.out.print(hoodEncoder.getPosition());
        //      System.out.print(", target: ");
        //     System.out.print(targetHoodAngle);
        //     System.out.print(", current: ");
        //     System.out.print(hoodMotor.getOutputCurrent());
        //     System.out.println("Moving hood up");
        //     if (Math.abs(hoodEncoder.getVelocity()) < 0.01) {
        //         System.out.println("Hood is stalled, kicking motor");
        //         hoodMotor.set(0.4);
        //     } else {
        //         hoodMotor.set(0.1);
        //     }
        // } else {
        //     hoodMotor.set(0);
        // }


        SmartDashboard.putNumber("Hood Pos", hoodEncoder.getPosition());

        if (isRunning) {
            targetSpeed = SmartDashboard.getNumber("Shooter Target Speed", 0);
            System.out.println("Target shooter speed: " + targetSpeed);
            shooterSpeedPID.setSetpoint(-interpolatedShooterSpeed);
            //shooterSpeedPID.setSetpoint(-interpolatedShooterSpeed);
            double output = shooterSpeedPID.calculate(topMotor.getEncoder().getVelocity());
            SmartDashboard.putNumber("Shooter Speed", topMotor.getEncoder().getVelocity());
            output = MathUtil.clamp(output, -1, 0);
            output = (prevOutput * Constants.ShooterConstants.kShooterFF) + (1-Constants.ShooterConstants.kShooterFF) * output;
            prevOutput = output;
            topMotor.set(output);
            bottomMotor.set(-output);
            
        }
        else {
            topMotor.set(0);
            bottomMotor.set(0);
        }
    }

    public void setMotorSpeed(double speed) {
        this.targetSpeed = speed;
    }

    public boolean getIsRunning() {
        return isRunning;
    }

    public void setIsRunning(boolean isRunning) {
        this.isRunning = isRunning;
    }

    public void toggleIsRunning() {
        System.out.println("shooter running" + !isRunning);
        isRunning = !isRunning;
    }

    public void toggleFullSpeed() {
        isFullSpeed = !isFullSpeed;
    }

    public void setHoodAngle(double angle) {
        this.targetHoodAngle = angle;
        System.out.println("Setting hood angle to " + angle);
        
    }
}
