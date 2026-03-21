package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HubTimer {

    private String ourAlliance = "";

    public HubTimer() {
        SmartDashboard.putBoolean("isHubActive", true);
        SmartDashboard.putString("Seconds Till Active", "Hub Active");
        updateAlliance();
    }

    public void updateAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                ourAlliance = "R";
            } else {
                ourAlliance = "B";
            }
        } else {
            SmartDashboard.putBoolean("isHubActive", false);
            SmartDashboard.putString("Seconds Till Active", "No Alliance Selected");
        }
    }

    public void updateElastic() {
        System.out.println("Running updateElastic");

        if (DriverStation.isAutonomousEnabled()) {
            // Hub is always active in autonomous
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putString("Seconds Till Active", "Hub Active");
            return;
        }

        boolean weActiveFirst; // Match time is number of seconds till game ends (not including auto, including
                               // endgame)
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        if (String.valueOf(gameData.charAt(0)) == ourAlliance) {
            // If we win auto, the gameData will be equal to ourAlliance and so we will NOT
            // be active first
            weActiveFirst = false;
        } else {
            // If we lose auto, the gameData will reflect that the other alliance is
            // inactive first
            // So WE will be active first
            weActiveFirst = true;
        }

        // NOTE: All of teleop is 140 seconds
        if (matchTime > 130) {
            // First 10 seconds is transition, hub active
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putString("Seconds Till Active", "Hub Active");
        } else if (matchTime > 105) {
            // Next 25 seconds First phase/shift. Won auto, inactive. Lost auto, active.
            SmartDashboard.putBoolean("isHubActive", weActiveFirst);
            if (weActiveFirst) {
                SmartDashboard.putString("Seconds Till Active", "Hub Active");
            } else {
                SmartDashboard.putString("Seconds Till Active",
                        "Hub Active in " + (int) (matchTime - 105) + " Seconds");
            }
        } else if (matchTime > 80) {
            // Next 25 seconds. Second phase/shift. Won auto, active. Lost auto, inactive.
            SmartDashboard.putBoolean("isHubActive", !weActiveFirst);
            if (!weActiveFirst) {
                SmartDashboard.putString("Seconds Till Active", "Hub Active");
            } else {
                SmartDashboard.putString("Seconds Till Active", "Hub Active in " + (int) (matchTime - 80) + " Seconds");
            }
        } else if (matchTime > 55) {
            // Next 25 seconds. Third phase/shift. Won auto, inactive. Lost auto, active.
            SmartDashboard.putBoolean("isHubActive", weActiveFirst);
            if (weActiveFirst) {
                SmartDashboard.putString("Seconds Till Active", "Hub Active");
            } else {
                SmartDashboard.putString("Seconds Till Active", "Hub Active in " + (int) (matchTime - 55) + " Seconds");
            }
        } else if (matchTime > 30) {
            // Next 25 seconds. Fourth phase/shift. Won auto, active. Lost auto, inactive.
            SmartDashboard.putBoolean("isHubActive", !weActiveFirst);
            if (!weActiveFirst) {
                SmartDashboard.putString("Seconds Till Active", "Hub Active");
            } else {
                SmartDashboard.putString("Seconds Till Active", "Hub Active in " + (int) (matchTime - 30) + " Seconds");
            }
        } else {
            // Last 30 seconds is endgame, hub is active.
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putString("Seconds Till Active", "Hub Active");
        }
    }
}
