package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubTimer extends SubsystemBase {

    private String ourAlliance = "";

    public HubTimer() {
        SmartDashboard.putBoolean("isHubActive", true);
        SmartDashboard.putNumber("Seconds Till Change", 0);
        setAlliance();
    }

    @Override
    public void periodic() {
        if (ourAlliance == "") {
            setAlliance();
        }
        updateElastic();
    }

    public void setAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                ourAlliance = "R";
            } else {
                ourAlliance = "B";
            }
        }
    }

    public void updateElastic() {

        if (DriverStation.isAutonomousEnabled()) {
            // Hub is always active in autonomous
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putNumber("Seconds Till Change", 0);
        }

        boolean weActiveFirst; // Match time is number of seconds till game ends (not including auto, including
                               // endgame)
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length() > 0 && String.valueOf(gameData.charAt(0)) == ourAlliance) {
            // If we win auto, the gameData will be equal to ourAlliance and so we will NOT
            // be active first
            weActiveFirst = false;
        } else {
            // If we lose auto, the gameData will reflect that the other alliance is
            // inactive first
            // So WE will be active first
            weActiveFirst = true;
        }

        SmartDashboard.putBoolean("Won Auto", !weActiveFirst);
        SmartDashboard.putBoolean("Our Hub First", weActiveFirst);

        // NOTE: All of teleop is 140 seconds
        if (matchTime > 130) {
            // First 10 seconds is transition, hub active
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putNumber("Seconds Till Change", 0);
        } else if (matchTime > 105) {
            // Next 25 seconds First phase/shift. Won auto, inactive. Lost auto, active.
            SmartDashboard.putBoolean("isHubActive", weActiveFirst);
            SmartDashboard.putNumber("Seconds Till Change", (int) (matchTime - 105));
        } else if (matchTime > 80) {
            // Next 25 seconds. Second phase/shift. Won auto, active. Lost auto, inactive.
            SmartDashboard.putBoolean("isHubActive", !weActiveFirst);
            SmartDashboard.putNumber("Seconds Till Change", (int) (matchTime - 80));
        } else if (matchTime > 55) {
            // Next 25 seconds. Third phase/shift. Won auto, inactive. Lost auto, active.
            SmartDashboard.putBoolean("isHubActive", weActiveFirst);
            SmartDashboard.putNumber("Seconds Till Change", (int) (matchTime - 55));
        } else if (matchTime > 30) {
            // Next 25 seconds. Fourth phase/shift. Won auto, active. Lost auto, inactive.
            SmartDashboard.putBoolean("isHubActive", !weActiveFirst);
            SmartDashboard.putNumber("Seconds Till Change", (int) (matchTime - 30));
        } else {
            // Last 30 seconds is endgame, hub is active.
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putNumber("Seconds Till Change", matchTime);
        }
    }
}
