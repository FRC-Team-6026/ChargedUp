package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTableInstance _instance = NetworkTableInstance.getDefault();
    private final NetworkTable _table = _instance.getTable("limelight");

    public boolean isTargets(){
        if(_table.getEntry("<tv>").getDouble(0) == 1){
            return true;
        } else {
            return false;
        }
    }

    public Pose2d getRobotPoseInTargetSpace() {
        var robotPoseArray = new double[6];
        _table.getEntry("botpose_targetspace").getDoubleArray(robotPoseArray);
        //target space from the perspective of looking at the target:
        //+X to the right of the target
        //+Y down to the ground
        //+Z straight out from the target
        var x = robotPoseArray[0];
        var y = robotPoseArray[2];
        var rotation = robotPoseArray[4];
        return new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
    }
}
