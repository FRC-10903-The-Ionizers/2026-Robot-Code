package frc.robot.DrivingCommands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DrivingCommands.DriveToPose;
import frc.robot.subsystems.Swerve;

public class GoToNearestTrench extends Command {
    private final Swerve swerve;
    private final boolean isRed;

    private final Pose2d topTrenchLocation;
    private final Pose2d bottomTrenchLocation;

    public GoToNearestTrench(Swerve swerve) {
        this.swerve = swerve;

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                isRed = true;
                topTrenchLocation = DriveConstants.kRedTopTrenchLocation;
                bottomTrenchLocation = DriveConstants.kRedBottomTrenchLocation;
            } else {
                isRed = false;
                topTrenchLocation = DriveConstants.kBlueTopTrenchLocation;
                bottomTrenchLocation = DriveConstants.kRedBottomTrenchLocation;
            }
        } else {
            System.out.println("SET ALLIANCE IN DRIVER STATION");
            isRed = false;
            topTrenchLocation = new Pose2d();
            bottomTrenchLocation = new Pose2d();
        }
    }

    @Override
    public void execute() {

    }

    public Command driveToNearestTrench(){
        Pose2d currentPose = swerve.getPose();

        double distToTopTrench = currentPose.getTranslation().getDistance(topTrenchLocation.getTranslation());
        double distToBottomTrench = currentPose.getTranslation().getDistance(bottomTrenchLocation.getTranslation());

        if(distToTopTrench > distToBottomTrench){
            return new DriveToPose(swerve, topTrenchLocation);
        } else {
            return new DriveToPose(swerve, bottomTrenchLocation);
        }
    }
}
