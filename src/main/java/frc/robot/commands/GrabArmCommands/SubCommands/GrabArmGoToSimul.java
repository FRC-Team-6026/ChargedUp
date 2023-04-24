// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands.SubCommands;

import frc.robot.subsystems.GrabArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Simultaniously moves Rotation and Extension torwards position at the same time. */
public class GrabArmGoToSimul extends CommandBase {

  private final GrabArm _Arm;
  private final GrabArm.GrabArmPositions _desiredPosition;

  private boolean _rotationStatus = false;
  private boolean _extensionStatus = false;


  public GrabArmGoToSimul(GrabArm Arm, GrabArm.GrabArmPositions desiredPosition) {
    _Arm = Arm;
    _desiredPosition = desiredPosition;
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Arm.setDesiredRotation(_desiredPosition);
    _Arm.setDesiredExtension(_desiredPosition);
    _Arm.desiredRotationToStationary(_desiredPosition);
    _Arm.desiredExtensionToStationary(_desiredPosition);
    _Arm.desiredRotationToTarget(_desiredPosition);
    _Arm.desiredExtensionToTarget(_desiredPosition);
    _Arm.calculateProfiles();
    _Arm.runCommandTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Arm.compensationComputation();
    if(_Arm.checkRotation(_desiredPosition)){
      _rotationStatus = true;
      _Arm.stationaryRotation();
    } else {
      _rotationStatus = false;
      _Arm.goToDesiredRotationPosition();
    }
    if(_Arm.checkExtension(_desiredPosition)){
      _extensionStatus = true;
      _Arm.stationaryExtension();
    } else {
      _extensionStatus = false;
      _Arm.goToDesiredExtensionPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Arm.stopNClearTimer();
    _Arm.comingFromCommand();
    _Arm.desiredRotationToStationary(_desiredPosition);
    _Arm.desiredExtensionToStationary(_desiredPosition);
    _Arm.desiredRotationToTarget(_desiredPosition);
    _Arm.desiredExtensionToTarget(_desiredPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_rotationStatus && _extensionStatus){
      return true;
    } else {return false;}
  }
}
