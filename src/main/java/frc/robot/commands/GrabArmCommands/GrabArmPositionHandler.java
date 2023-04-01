// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands;

import frc.robot.commands.GrabArmCommands.SubCommands.GrabArmGoToExtension;
import frc.robot.commands.GrabArmCommands.SubCommands.GrabArmGoToRotation;
import frc.robot.subsystems.GrabArm;
import frc.robot.subsystems.GrabArm.GrabArmPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class GrabArmPositionHandler {
  /** Example static factory for an autonomous command. */
  public static CommandBase PositionHandler(GrabArm Arm, GrabArm.GrabArmPositions Position){
    if(Position.equals(GrabArmPositions.Stow)){
      return StowPosition(Arm, Position);
    }
    return MainPositions(Arm, Position);
  }

  public static CommandBase StowPosition(GrabArm Arm, GrabArm.GrabArmPositions Position) {
    return Commands.sequence(
      new GrabArmGoToExtension(Arm, Position),
      new GrabArmGoToRotation(Arm, Position)
    );
  }

  public static CommandBase MainPositions(GrabArm Arm, GrabArm.GrabArmPositions Position) {
    var CommandSequence = Commands.sequence();

    if(Arm.checkNeedToLimitTravelExtension()){
      CommandSequence.andThen(new GrabArmGoToExtension(Arm, GrabArmPositions.TravelLimit));
    }

    return CommandSequence.andThen(
      new GrabArmGoToRotation(Arm, Position)
    ).andThen(
      new GrabArmGoToExtension(Arm, Position)
    );
    

  }
}
