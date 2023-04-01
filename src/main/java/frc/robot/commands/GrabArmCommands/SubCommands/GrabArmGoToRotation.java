// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands.SubCommands;

import frc.robot.subsystems.GrabArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GrabArmGoToRotation extends CommandBase {

  private final GrabArm _Arm;
  private final GrabArm.GrabArmPositions _targetRotation;

  public GrabArmGoToRotation(GrabArm Arm, GrabArm.GrabArmPositions targetRotation) {
    _Arm = Arm;
    _targetRotation = targetRotation;
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Arm.setDesiredRotation(_targetRotation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Arm.stationaryExtension();
    _Arm.goToDesiredRotation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Arm.currentRotationToStaionary();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _Arm.checkRotation(_targetRotation);
  }
}
