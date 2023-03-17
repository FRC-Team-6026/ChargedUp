package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry _swerveOdometry;
  private SwerveModule[] _SwerveMods;

  private boolean _fieldAngleAdjusted = false;
  private boolean _calibratingGyro = false;

  private Timer _timer;

  private Field2d _field;

  public Swerve() {
    gyro = new AHRS();
    gyro.reset();
    zeroGyro();

    _SwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    _swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

    _field = new Field2d();
    SmartDashboard.putData("Field", _field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if(!_calibratingGyro){
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getAngle())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : _SwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      var modState = swerveModuleStates[mod.moduleNumber];
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired angle: ", modState.angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " desired velocity: ", modState.speedMetersPerSecond);
    }
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : _SwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return _swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    _swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : _SwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : _SwerveMods) {
      positions[mod.moduleNumber] = mod.getPostion();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(0);
    _fieldAngleAdjusted = false;
  }

  public void zeroFieldAngleOffset(double angleOffset, boolean isYourSide){ //isYourSide will be used for detecting if a 180 offset is to be added to the angle, to catch if you zero it on a tag on the opposing side from where automated proceedures should take place (i.e. loading zone)
    gyro.zeroYaw();
    if(isYourSide){
      gyro.setAngleAdjustment(angleOffset);
    } else {
      gyro.setAngleAdjustment(180 + angleOffset);
    }
    _fieldAngleAdjusted = true;
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Rotation2d getAngle() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getAngle())
        : Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void resetToAbsolute() {
    for (SwerveModule mod : _SwerveMods) {
        mod.resetToAbsolute();
    }
  }

  public void updateGyroCalibration() {
    _calibratingGyro = true;
    gyro.calibrate();
    _timer.start();
    while(_calibratingGyro){
      if(_timer.hasElapsed(2)){
        _timer.stop();
        _timer.reset();
        _calibratingGyro = false;
      }
    }
  }

  public void fieldUpdate(Pose2d newPose) {
    _field.setRobotPose(newPose);
  }

  @Override
  public void periodic() {
    if(_fieldAngleAdjusted){
      _swerveOdometry.update(getAngle(), getPositions());
    } else {
      _swerveOdometry.update(getYaw(), getPositions());
    }
    _field.setRobotPose(getPose());

    SmartDashboard.putBoolean("isAngleAdjusted", _fieldAngleAdjusted);

    for (SwerveModule mod : _SwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);      
    }
  }
}