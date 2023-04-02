// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands.SubCommands;

import frc.robot.subsystems.GrabArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class GrabArmGoToSimul extends CommandBase {

  private final GrabArm _Arm;
  private final GrabArm.GrabArmPositions _desiredRotation;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Arm.compensationComputation();
    _Arm.goToDesiredExtension();
    _Arm.goToDesiredRotation();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Arm.desiredRotationToStationary(_desiredRotation);
    _Arm.desiredExtensionToStationary(_desiredRotation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(_Arm.checkRotation(_desiredRotation) && _Arm.checkExtension(_desiredRotation)){
      return true;
    } else {return false;}
  }
}
