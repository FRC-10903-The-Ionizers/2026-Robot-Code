package frc.robot.commands.drivingcommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import java.util.Set;

public class DriveToPoint {

  private static final PathConstraints CONSTRAINTS = new PathConstraints(
      3.0,                              // max vel (m/s)
      3.0,                              // max accel (m/s²)
      Units.degreesToRadians(540),      // max ang vel
      Units.degreesToRadians(720));     // max ang accel

  public static Command create(Swerve swerve, Pose2d targetPose) {
    return Commands.defer(
        () -> AutoBuilder.pathfindToPose(
            targetPose,
            CONSTRAINTS,
            0.0),
        Set.of(swerve));
  }
}