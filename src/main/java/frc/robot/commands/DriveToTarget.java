package frc.robot.commands;

import java.util.List;

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
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DriveToTarget extends SequentialCommandGroup {
    /**
     * 
     * @param swerve
     * @param limelight
     * @param position -1 align to cones left of target, 0 align with target, 1 align right of target
     */
    public DriveToTarget(Swerve swerve, Limelight limelight, int position) {
        super();

        var robotPose = limelight.getRobotPoseInTargetSpace();

        TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);

        var x = 0.0;
        if (position == -1) {
            x = -0.5;
        } else if (position == 1) {
            x = 0.5;
        }

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectoryToTarget = TrajectoryGenerator.generateTrajectory(
            // Start at the robot position relative to the target in view
            robotPose,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(x, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(x, 0.5, Rotation2d.fromDegrees(0)),
            config);

        var thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectoryToTarget,
            swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);

        addCommands(new InstantCommand(() -> swerve.resetOdometry(trajectoryToTarget.getInitialPose())),
            swerveControllerCommand);
    }
}
