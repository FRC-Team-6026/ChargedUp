package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.LevelRobot;
import frc.robot.commands.GrabArmCommands.GrabArmPositionHandler;
import frc.robot.subsystems.GrabArm;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.GrabArm.GrabArmPositions;

public class TopConeGrabCubeTopCubeTop extends SequentialCommandGroup {

  public TopConeGrabCubeTopCubeTop(Swerve s_Swerve, GrabArm _Arm) {
    
    addRequirements(s_Swerve);
    addRequirements(_Arm);
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("TopConeGrabCubeTopCubeTop", new PathConstraints(4, 8));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("openGrabber", new InstantCommand(() -> _Arm.openGrabber()));
    eventMap.put("closeGrabber", new InstantCommand(() -> _Arm.closeGrabber()));
    eventMap.put("stow", GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.Stow, true));
    eventMap.put("floor", GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.Floor, true));
    eventMap.put("topCone", GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.TopCone, true));
    eventMap.put("topCube", GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.TopCube, true));
    eventMap.put("midCone", GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.MidCone, true));
    eventMap.put("midCube", GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.MidCube, true));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, // Pose2d supplier
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );
    addCommands(new InstantCommand(() -> _Arm.closeGrabber()));
    addCommands(new InstantCommand(() -> s_Swerve.xPatternFalse()));
    addCommands(new InstantCommand(() -> s_Swerve.invertGyro()));
    addCommands(GrabArmPositionHandler.PositionHandler(_Arm, GrabArmPositions.TopCone, true));
    addCommands(new InstantCommand(() -> _Arm.openGrabber()));
    addCommands(autoBuilder.fullAuto(pathGroup));
    addCommands(new LevelRobot(s_Swerve));
  }
}