package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.idProfiles.Id1;
import frc.robot.autos.idProfiles.Id2;
import frc.robot.autos.idProfiles.Id3;
import frc.robot.autos.idProfiles.Id6;
import frc.robot.autos.idProfiles.Id7;
import frc.robot.autos.idProfiles.Id8;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;

import java.util.Hashtable;

public class ExampleAuto {


private final Hashtable<Integer,Command> idCommands = new Hashtable<Integer,Command>();

    public ExampleAuto() {
        idCommands.put(1,Id1.getcommand());
        idCommands.put(2,Id2.getcommand());
        idCommands.put(3,Id3.getcommand());
        idCommands.put(6,Id6.getcommand());
        idCommands.put(7,Id7.getcommand());
        idCommands.put(8,Id8.getcommand());
    }

    public Command getCommand(){
        var limeLight = new Limelight();
        var tagId = limeLight.getTagId();
        var tagIdInt = (int)tagId;
        var tagIdInteger = (Integer)tagIdInt;
        return idCommands.get(tagIdInteger);
    }
}