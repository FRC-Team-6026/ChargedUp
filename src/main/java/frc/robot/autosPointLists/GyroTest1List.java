package frc.robot.autosPointLists;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroTest1List {
    public ArrayList<Pose2d> gyroList;
    
    public GyroTest1List(){
        gyroList = new ArrayList<Pose2d>();
        gyroList.add(new Pose2d(0, 0, new Rotation2d(0)));
        gyroList.add(new Pose2d(1, 0, new Rotation2d(0)));
    }

    public ArrayList<Pose2d> GyroTest1ListRetrieve(Pose2d initial){
        gyroList.set(0, initial);
        return gyroList;
    }
}
