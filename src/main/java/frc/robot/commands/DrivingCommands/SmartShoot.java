package frc.robot.commands.DrivingCommands;

import java.util.Optional;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class SmartShoot extends Command {
    /**
     * Assume that the robot is already facing correct hub
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Shooter shooter;
    private Hopper hopper;
    private final InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix;

    public SmartShoot(Swerve swerve, Shooter shooter, Hopper hopper,
            InterpolatingMatrixTreeMap<Double, N2, N1> interpolationMatrix) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve:     the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.shooter = shooter;
        this.hopper = hopper;
        this.interpolationMatrix = interpolationMatrix;
        addRequirements(hopper, shooter);
    }

    @Override
    public void execute() {
        /**
         * Method that drives the swerves
         */
        // Get target translation

        Optional<Alliance> ally = DriverStation.getAlliance();

        if (!ally.isPresent()) {
            System.out.println("Alliance not detected!");
        }
        Translation2d targetTranslation;
        if (ally.get() == Alliance.Red && swerve.getPose().getTranslation().getX() > 11.8) {
            targetTranslation = Constants.ShooterConstants.kRedHubTranslation;
        } else if (ally.get() == Alliance.Blue && swerve.getPose().getTranslation().getX() < 4.6) {
            targetTranslation = Constants.ShooterConstants.kBlueHubTranslation;
        } else {
            double targetPoseY;
            if (this.swerve.getPose().getY() >= 4.035) {
                targetPoseY = (4.035 / 2) * 3;
            } else {
                targetPoseY = 4.035 / 2;
            }

            if (ally.get() == Alliance.Red) {
                targetTranslation = new Translation2d(14.4, targetPoseY);
            } else {
                targetTranslation = new Translation2d(2.3, targetPoseY);
            }
        }

        SmartDashboard.putNumber("Distance to Hub", (swerve.getPose().getTranslation().getDistance(targetTranslation)));
        double distanceToTarget = swerve.getPose().getTranslation().getDistance(targetTranslation);

        // With distance start getting shooting stuff set
        Matrix<N2, N1> interp = interpolationMatrix.get(distanceToTarget);
        double interpolatedShooterSpeed = 0.0;
        double interpolatedHoodAngle = 0.0;

        if (interp != null) {
            interpolatedShooterSpeed = interp.get(0, 0);
            interpolatedHoodAngle = interp.get(1, 0);
        }

        this.shooter.setSpeedAndAngle(interpolatedShooterSpeed, interpolatedHoodAngle);
        if ((this.shooter.getIsReadyToShoot())) {
            hopper.setIsHopperRunning(true);
        } else {
            hopper.setIsHopperRunning(false);
        }
    }
}