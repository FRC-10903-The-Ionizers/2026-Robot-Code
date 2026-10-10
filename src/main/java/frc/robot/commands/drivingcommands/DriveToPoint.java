package frc.robot.commands.drivingcommands;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.json.JSONArray;
import org.json.JSONObject;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class DriveToPoint {

    private static final PathConstraints CONSTRAINTS = new PathConstraints(
        0.25,                       // max vel (m/s) originally 1
        0.25,                 // max accel (m/s²) originally 3
        Units.degreesToRadians(540),      // max ang vel
        Units.degreesToRadians(720));     // max ang accel

    public static Command createGoToPointCommand(Swerve swerve, Pose2d targetPose) {
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(
                targetPose,
                CONSTRAINTS,
                0.0),
            Set.of(swerve)
        );
    }

    public static Command createPath(Swerve swerve, JSONObject pathJSON) {
        List<Command> commandList = new ArrayList<>();
        commandList.add(Commands.print("Command Started"));
        try {
            JSONArray waypoints = pathJSON.getJSONArray("waypoints");

            for (int i = 0; i < waypoints.length(); i++) {
                JSONObject waypoint = waypoints.getJSONObject(i);
                
                double x = waypoint.getDouble("x");
                double y = waypoint.getDouble("y");
                double rotation = waypoint.getDouble("rot");
                
                Pose2d targetPose = new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(rotation));

                Command action = Commands.none();
                if (waypoint.has("event")) {
                    String event = waypoint.getString("event");
                    Command foundCommand = NamedCommands.getCommand(event);

                    if (foundCommand != null) {
                        action = foundCommand;
                        
                        if (waypoint.has("duration")) {
                            action = action.withTimeout(waypoint.getDouble("duration"));
                        }
                    } else {
                        System.err.println("That event is not added in NamedCommands: " + event);
                    }
                }
                
                Command goToPointCommand = createGoToPointCommand(swerve, targetPose);
                boolean stop = waypoint.has("stop") && waypoint.getBoolean("stop");
                
                Command waypointStep = stop 
                    ? goToPointCommand.andThen(action) 
                    : Commands.parallel(goToPointCommand, action);

                commandList.add(waypointStep);
            }
            return Commands.sequence(commandList.toArray(new Command[0]));
        } catch (Exception e) {
            System.err.println("Spooky... somethings wrong with JSON " + e.getMessage());
            return Commands.none();
        }
    }
}