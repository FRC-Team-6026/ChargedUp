// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands.SubCommands;

import frc.robot.subsystems.GrabArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Simultaniously moves Rotation and Extension torwards position at the same time. */
public class GrabArmGoToSimul extends CommandBase {

  private final GrabArm _Arm;
  private final GrabArm.GrabArmPositions _desiredRotation;

  private boolean _rotationStatus = false;
  private boolean _extensionStatus = false;


  public GrabArmGoToSimul(GrabArm Arm, GrabArm.GrabArmPositions desiredRotation) {
    _Arm = Arm;
    _desiredRotation = desiredRotation;
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Arm.setDesiredRotation(_desiredRotation);
    _Arm.setDesiredExtension(_desiredRotation);
    _Arm.desiredRotationToStationary(_desiredRotation);
    _Arm.desiredExtensionToStationary(_desiredRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Arm.compensationComputation();
    if(_Arm.checkRotation(_desiredRotation)){
      _rotationStatus = true;
      _Arm.stationaryRotation();
    } else {
      _rotationStatus = false;
      _Arm.goToDesiredRotation();
    }
    if(_Arm.checkExtension(_desiredRotation)){
      _extensionStatus = true;
      _Arm.stationaryExtension();
    } else {
      _extensionStatus = false;
      _Arm.goToDesiredExtension();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_rotationStatus && _extensionStatus){
      return true;
    } else {return false;}
  }
}
