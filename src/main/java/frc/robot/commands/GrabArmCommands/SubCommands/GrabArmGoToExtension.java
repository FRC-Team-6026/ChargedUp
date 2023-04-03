// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabArmCommands.SubCommands;

import frc.robot.subsystems.GrabArm;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Sets the ArmExtension to the desired position while holding rotation stattionary **/
public class GrabArmGoToExtension extends CommandBase {

  private final GrabArm _Arm;
  private final GrabArm.GrabArmPositions _desiredExtension;

  public GrabArmGoToExtension(GrabArm Arm, GrabArm.GrabArmPositions desiredExtension) {
    _Arm = Arm;
    _desiredExtension = desiredExtension;
    addRequirements(Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _Arm.setDesiredExtension(_desiredExtension);
    _Arm.calculateProfiles();
    _Arm.runCommandTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _Arm.compensationComputation();
    _Arm.stationaryRotation();
    _Arm.goToDesiredExtensionPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _Arm.desiredExtensionToStationary(_desiredExtension);
    _Arm.stopNClearTimer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _Arm.checkExtension(_desiredExtension);
  }
}
