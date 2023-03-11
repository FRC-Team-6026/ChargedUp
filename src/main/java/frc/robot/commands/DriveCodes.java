package frc.robot.commands;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class DriveCodes {

    double pitch;
    double tolerance = Constants.Swerve.maxPitchDegrees;
    double incrimentMeters = Constants.Swerve.incrimentalChangeMeters;

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        Trajectory forward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(incrimentMeters/2, 0)), 
            new Pose2d(incrimentMeters, 0, new Rotation2d(0)), 
            config);
        
        Trajectory reverse = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(-incrimentMeters/2, 0)), 
            new Pose2d(-incrimentMeters, 0, new Rotation2d(0)), 
            config);

        ProfiledPIDController thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController,
                0,
                0,
                Constants.AutoConstants.kThetaControllerConstraints);

        SwerveControllerCommand swerveControllerForward;
        SwerveControllerCommand swerveControllerReverse;
    
    public void levelingCode(frc.robot.subsystems.Swerve s_Swerve, AHRS gyro) {
        
        pitch = MathUtil.applyDeadband(gyro.getPitch(), tolerance);
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerReverse =
            new SwerveControllerCommand(
                reverse,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand swerveControllerForward =
            new SwerveControllerCommand(
                forward,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        while(frc.robot.subsystems.Swerve.leveling){
            do {
            updatePitch(gyro);
            swerveControllerReverse.execute();
            updatePitch(gyro);
            } while (pitch > 0);

            do {
            updatePitch(gyro);
            swerveControllerForward.execute();
            updatePitch(gyro);
            } while (pitch < 0);

            do {
                updatePitch(gyro);
            } while (pitch == 0);

        }
      }

      private void updatePitch(AHRS gyro){
        pitch = MathUtil.applyDeadband(gyro.getPitch(), tolerance);
      }
}
