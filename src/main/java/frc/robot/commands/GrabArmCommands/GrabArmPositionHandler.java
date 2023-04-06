// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands;

import frc.robot.commands.GrabArmCommands.SubCommands.GrabArmGoToExtension;
import frc.robot.commands.GrabArmCommands.SubCommands.GrabArmGoToRotation;
import frc.robot.commands.GrabArmCommands.SubCommands.GrabArmGoToSimul;
import frc.robot.subsystems.GrabArm;
import frc.robot.subsystems.GrabArm.GrabArmPositions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class GrabArmPositionHandler extends CommandBase{
  /** Example static factory for an autonomous command. */
  public static CommandBase PositionHandler(GrabArm Arm, GrabArm.GrabArmPositions Position, boolean simultanous){
    
    if(simultanous){
    return SimulMovementPositions(Arm, Position);
    } else {
      if(Position.equals(GrabArmPositions.Stow)){
        return StowPosition(Arm, Position);
      }
      return MainPositions(Arm, Position); 
    }
  }

  private static CommandBase StowPosition(GrabArm Arm, GrabArm.GrabArmPositions Position) {
    return Commands.sequence(
      new GrabArmGoToExtension(Arm, Position),
      new GrabArmGoToRotation(Arm, Position)
    );
  }

  private static CommandBase MainPositions(GrabArm Arm, GrabArm.GrabArmPositions Position) {
    if(Arm.checkNeedToLimitTravelExtension()){
      return Commands.sequence(
        new GrabArmGoToExtension(Arm, GrabArmPositions.TravelLimit),
        new GrabArmGoToRotation(Arm, Position),
        new GrabArmGoToExtension(Arm, Position)
      );
    } else {
    return Commands.sequence(
        new GrabArmGoToRotation(Arm, Position),
        new GrabArmGoToExtension(Arm, Position)
      );
    }
  }

  private static CommandBase SimulMovementPositions(GrabArm Arm, GrabArm.GrabArmPositions Position){
    return new GrabArmGoToSimul(Arm, Position);
  }
}
