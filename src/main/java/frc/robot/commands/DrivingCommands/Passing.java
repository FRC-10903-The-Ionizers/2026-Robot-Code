// package frc.robot.commands.DrivingCommands;

// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import java.util.Optional;

// import com.revrobotics.spark.SparkMax;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.statesensors.RegionHandler;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Swerve;

// public class Passing extends Command {

//     private final SparkMax topMotor = new SparkMax(Constants.ShooterConstants.kTopMotorID, MotorType.kBrushless);
//     private final SparkMax bottomMotor = new SparkMax(Constants.ShooterConstants.kBottomMotorID, MotorType.kBrushless);
//     private final SparkMax spindexerMotor = new SparkMax(Constants.HopperConstants.kSpindaxerMotorID,
//             MotorType.kBrushless);
//     private final SparkMax bobMotor = new SparkMax(Constants.HopperConstants.kBobMotorID, MotorType.kBrushless);
//     private final ShootCommand shootCommand;
//     private final RegionHandler regionHandler = new RegionHandler("regions.json");
//     private final Swerve swerve;
//     private final double targetPoseX;
//     private final Pose2d targetHubPose;

//     private

//     public Passing(Shooter shooter, Swerve swerve) {
//         shootCommand = new ShootCommand(shooter);
//         this.swerve = swerve;

//         Optional<Alliance> ally = DriverStation.getAlliance();

//         if (ally.isPresent()) {
//             if (ally.get() == Alliance.Red) {
//                 targetPoseX = 4.62;
//             } else {
//                 targetPoseX = 11.92;
//             }
//         } else {
//             throw new Error("SELECT ALLIANCE IN DRIVER STATION");
//         }
//     }

//     @Override
//     public void execute() {
//         double targetPoseY;
//         if (this.swerve.getPose().getY() >= 4.035) {
//             targetPoseY = (4.035 / 2) * 3;
//         } else {
//             targetPoseY = 4.035 / 2;
//         }
//         Pose2d targetPose = new Pose2d(new Translation2d(targetPoseX, targetPoseY), new Rotation2d());

//         topMotor.set(-1);
//         bottomMotor.set(-1);
//         spindexerMotor.set(-0.75);
//         bobMotor.set(0.75);
//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//     }
// }
