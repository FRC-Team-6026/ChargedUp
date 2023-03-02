// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final XboxController driver = new XboxController(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kBack.value);
  private final JoystickButton robotCentricBumper =
      new JoystickButton(driver, XboxController.Button.kStart.value);
  private final JoystickButton openGrabber =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton closeGrabber =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton driveToTargetCenter =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driveToTargetLeft =
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton driveToTargetRight =
      new JoystickButton(driver, XboxController.Button.kB.value);
  private boolean robotCentric = false;
  /* Subsystems */
  private final Swerve _swerve = new Swerve();
  private final GrabArm _grabArm = new GrabArm();
  private final Limelight _limelight = new Limelight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    _swerve.setDefaultCommand(
        new TeleopSwerve(
            _swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric));

    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> _swerve.zeroGyro()));
    robotCentricBumper.onTrue(new InstantCommand(() -> {
      robotCentric = !robotCentric;
      SmartDashboard.putBoolean("Is Robot Centric", robotCentric);
    }));
    openGrabber.onTrue(new InstantCommand(() -> _grabArm.openGrabber()));
    closeGrabber.onTrue(new InstantCommand(() -> _grabArm.closeGrabber()));
    driveToTargetLeft.onTrue(new DriveToTarget(_swerve, _limelight, -1));
    driveToTargetCenter.onTrue(new DriveToTarget(_swerve, _limelight, 0));
    driveToTargetRight.onTrue(new DriveToTarget(_swerve, _limelight, 1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ExampleAuto(_swerve);
  }
}