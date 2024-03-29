package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.07;

    //Leveling Constants
    public static final double levelingMaxVelocityMs = 0.75;
    public static final double maxPitchDegrees = 2.5;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(18.75);
    public static final double wheelBase = Units.inchesToMeters(27.25);
    public static final double wheelDiameter = 98.5 / 1000.0; // mm to m
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0/7.0); // 150:7

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.3;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.3;
    public static final double driveKV = 2.55;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        wheelCircumference / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 5; // meters per second
    public static final double maxAngularVelocity = 4.5; //radians per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = true;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(144.05);
      public static final double xPosition = -45;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, xPosition);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(318.86);
      public static final double xPosition = 45;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, xPosition);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(323.96);
      public static final double xPosition = 45;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, xPosition);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(221.48);
      public static final double xPosition = -45;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, xPosition);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 8;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class GrabArm {
    public static final double spoolDiameter = 0.375;
    public static final double spoolRadius = spoolDiameter/2;
    public static final double firstStageTension = 10;
    public static final double secondStageTension = 5;
    public static final double tensionLesseningFactor = secondStageTension/firstStageTension;
    public static final double firstStageAprxWeight = 5;
    public static final double secondStageAprxWeight = 3;

    public static final double stickDeadband = 0.05;
    public static final double rotationGearRatio = 80;
    public static final double rotationConversionFactor = 360.0 / rotationGearRatio;
    public static final double rotationVelocityConversionFactorDps = rotationConversionFactor / 60.0;
    public static final double extensionGearRatio = 2.25;
    public static final double extensionConversionFactorInches = spoolDiameter * Math.PI / extensionGearRatio;
    public static final double extensionVelocityConversionFactorIps = extensionConversionFactorInches / 60.0;

    public static final int rotationContinuousCurrentLimit = 35;
    public static final int extensionContinuousCurrentLimit = 20;

    public static final boolean rotationInvert = false;
    public static final boolean extensionInvert = true;
    public static final IdleMode rotationNeutralMode = IdleMode.kBrake;
    public static final IdleMode extensionNeutralMode = IdleMode.kBrake;

    public static final double voltageComp = 12.0;

    public static final double rotationKP = 0.09;
    public static final double rotationKI = 0.0;
    public static final double rotationKD = 0.025;
    public static final double rotationKFF = 0.0;
    public static final double rotationMin = -.65;
    public static final double rotationMax = .65;

    public static final double rotationKPAuto = 0.012;
    public static final double rotationKIAuto = 0;
    public static final double rotationKDAuto = 0;
    public static final double rotationKFFAuto = 0;

    public static final double extensionKP = 0.07;
    public static final double extensionKI = 0.0;
    public static final double extensionKD = 0.0;
    public static final double extensionKFF = 0.0;
    public static final double extensionMin = -.35;
    public static final double extensionMax = .30;

    public static final double extensionKPAuto = 0.03;
    public static final double extensionKIAuto = 0;
    public static final double extensionKDAuto = 0.05;
    public static final double extensionKFFAuto = 0;


    public static final double codeExecutionRate = 50.0;
    public static final double codeExecutionRateTime = 1.0 / codeExecutionRate;

    public static final double maxRotationDps = 80.0;
    public static final double maxRotationAccDps = 45.0;
    public static final double maxRotationExecution = maxRotationDps / codeExecutionRate;
    public static final double maxRotationAccDpsExecution = maxRotationAccDps / codeExecutionRate;
    public static final double maxIps = 10.0;
    public static final double maxIpsAcc = 8.0;
    public static final double maxIpsExecution = maxIps / codeExecutionRate;
    public static final double maxIpsAccExecution = maxIpsAcc / codeExecutionRate;
    public static final double maxAutoPositionRotationDps = 125;
    public static final double maxAutoPositionRotationDpsAcc = 175;
    public static final double maxAutoPositionMaxIps = 15.0;
    public static final double maxAutoPositionMaxIpsAcc = 30.0;

    public static final float rotationForwardSoftLimitDegrees = 205;
    public static final float extensionForwardSoftLimitInches = 21;

    public static final double rotationOffsetinDegrees = 27;
    public static final double heightLimit = 72; // Max Height Minus 6''
    public static final double pivotHeightInches = 26.5;  
    public static final double baseArmLength = 32.5;
    public static final double maxExtensionHeight = heightLimit - pivotHeightInches;
    public static final double coneWeightLb = 1.4375;

    public static final double rotationPositionSettingToleranceDegrees = 2;
    public static final double extensionPositionSettingToleranceInches = 0.25;

    public static final double rotationStallTorque = 2.6;
    public static final double extensionStallTorque = 2.6;
  }
}