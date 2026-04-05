package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;




public class HubTimer extends SubsystemBase {

    private String ourAlliance = "";

    public HubTimer() {
        SmartDashboard.putBoolean("isHubActive", true);
        SmartDashboard.putNumber("Seconds Till Change", 0);
        setAlliance();
    }

    @Override
    public void periodic() {
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
        } else {
            System.out.println("NO ALLIANCE SELECTED IN DS!");
        }
    }

    public void updateElastic() {
        if (DriverStation.isAutonomousEnabled()) {
            //auto period, both hubs active
            SmartDashboard.putBoolean("isHubActive", true);
            SmartDashboard.putNumber("Seconds Till Change", DriverStation.getMatchTime());
        } else {
            //teleoperiod, only one hub active
            String firstInactiveAlliance = String.valueOf(DriverStation.getGameSpecificMessage().charAt(0));
            double timeLeftTeleop = Constants.kTotalTeleopTime - DriverStation.getMatchTime();

            if (timeLeftTeleop > 30) {
                SmartDashboard.putNumber("Seconds Till Change", (timeLeftTeleop - 30) % 20);
                if (firstInactiveAlliance.equals(ourAlliance)) {
                    // if the first inactive alliance is the same as our alliance, the hub is active if the number of shifts since teleop starting are even
                    SmartDashboard.putBoolean("isHubActive", ((int)((timeLeftTeleop - 30) / 20)) % 2 == 0);
                } else {
                    // if the first inactive alliance is not the same as our alliance, the hub is inactive
                    SmartDashboard.putBoolean("isHubActive", ((int)((timeLeftTeleop - 30) / 20)) % 2 == 1);
                }
            } else {
                SmartDashboard.putNumber("Seconds Till Change", timeLeftTeleop-110);
            }

        }
    }
}
