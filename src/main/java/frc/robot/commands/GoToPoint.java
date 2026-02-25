// ""package frc.robot.commands;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;
// /** An example command that uses an example subsystem. */
// public class GoToPoint extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final Swerve swerve;
//   private final Vision vision;
  
//   public GoToPoint(Swerve subsystem, Pose2d targetPose) {
//     swerve = subsystem;
//   }
//   public void initialize() {
//   }
//   public void execute() {
//     Pose2d currentPose = swerve.getPose();
//     double x_distance = targetPose.getX() - currentPose.getX();
//     double y_distance = targetPose.getY() - currentPose.getY();
//     double heading_distance = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
//     swerve.drive(x_distance, y_distance, heading_distance);
//   }
// }""