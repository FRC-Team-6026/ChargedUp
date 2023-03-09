package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;
import java.util.List;

public class ExampleAuto extends SequentialCommandGroup {
  public ExampleAuto(Swerve s_Swerve) {
    Limelight limeLight = new Limelight();
    double Id = limeLight.getTagId();

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    if(Id == 1 || Id == 2 || Id == 3) //Red Side of the Field, 1 being the farthest from the loading station
    {
        Pose2d startingPoseProOffset = limeLight.getRobotPoseInTargetSpace();
        startingPoseProOffset = new Pose2d(-startingPoseProOffset.getX(), -startingPoseProOffset.getY(), startingPoseProOffset.getRotation());
        if(Id == 1){
            
        }
        else if (Id == 2){

        }
        else if (Id == 3){

        }
    } 
    else if (Id == 8 || Id == 7 || Id == 6) //Blue Side of the Field, 8 being the farthest from the loading station
    {
        Pose2d startingPosePreOffset = limeLight.getRobotPoseInTargetSpace();
        Rotation2d invertRotation = startingPosePreOffset.getRotation();
        invertRotation = Rotation2d.fromDegrees(-invertRotation.getDegrees());
        startingPosePreOffset = new Pose2d(startingPosePreOffset.getX(), startingPosePreOffset.getY(), invertRotation);
        if (Id == 8){

        }
        else if (Id == 7){

        }
        else if (Id == 6){

        }
    }
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1,1), 
                    new Translation2d(2, -1),
                    new Translation2d(2,4)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand);
  }
}