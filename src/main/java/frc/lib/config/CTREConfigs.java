package frc.lib.config;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import frc.robot.Constants;

public final class CTREConfigs {
  public CANCoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
  }
}