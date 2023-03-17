package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GyroPositionDrive extends CommandBase{
    private final double _maxVelocityMs = Constants.AutoConstants.gyroMaxVelocity;
    private final double _tolerancePosition = Constants.AutoConstants.gyroPositionTolerance;
    private final double _toleranceRotation = Constants.AutoConstants.gyroRotationTolerance;
    private final double _meterRange = Constants.AutoConstants.gyroOutputMeterRange;
    private final double _degreeRange = Constants.AutoConstants.gyroOutputDegreeRange;

    private final double initialX;
    private final double initialY;
    private final double initialRotation;

    private final double targetX;
    private final double targetY;
    private final double targetDegrees;

    private double xValue;
    private double yValue;
    private double rotationValue;

    private final Swerve _swerve;
    private final AHRS _gyro;

    //MAKE SURE THAT 0 DEGREES IS STRAIGHT FORWARD
    public GyroPositionDrive(Swerve tempSwerve, Pose2d pose){
        _swerve = tempSwerve;
        _gyro = tempSwerve.getGyro();
        initialX = _gyro.getDisplacementX();
        initialY = _gyro.getDisplacementZ();
        initialRotation = _gyro.getYaw();

        targetX = pose.getX();
        targetY = pose.getY();
        targetDegrees = pose.getRotation().getDegrees();

        addRequirements(_swerve);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        xValue = getXRatio();
        yValue = getYRatio();
        rotationValue = getRotaitonRatio();
        _swerve.drive(new Translation2d(xValue,yValue).times(_maxVelocityMs), rotationValue, true, true);
    }

    @Override
    public boolean isFinished(){
        if(xValue == 0 && yValue == 0 && rotationValue == 0){
            return true;
        }
        return false;
    }

    private double getXRatio(){
        double position = _gyro.getDisplacementX() - initialX;
        if(MathUtil.applyDeadband(position, _tolerancePosition) == 0){
            return 0.0;
        }
        //calculating abs ratio 0 to 1
        double zeroToOne = (Math.abs(targetX - position)) / _meterRange;
        if (zeroToOne > 1) {
            zeroToOne = 1;
        } else if (zeroToOne < 0) {
            zeroToOne = 0;
        }
        //squaring to drop off input faster as approaching zero
        zeroToOne = zeroToOne * zeroToOne;
        if(position < 0){
            return -zeroToOne;
        } else {
            return zeroToOne;
        }
    }

    private double getYRatio(){
        double position = _gyro.getDisplacementZ() - initialY;
        if(MathUtil.applyDeadband(position, _tolerancePosition) == 0){
            return 0.0;
        }
        //calculating abs ratio 0 to 1
        double zeroToOne = (Math.abs(targetY - position)) / _meterRange;
        if (zeroToOne > 1) {
            zeroToOne = 1;
        } else if (zeroToOne < 0) {
            zeroToOne = 0;
        }
        //squaring to drop off input faster as approaching zero
        zeroToOne = zeroToOne * zeroToOne;
        if(position < 0){
            return -zeroToOne;
        } else {
            return zeroToOne;
        }
    }

    private double getRotaitonRatio(){
        double angle = _gyro.getYaw() - initialRotation;
        if(MathUtil.applyDeadband(angle, _toleranceRotation) == 0){
            return 0.0;
        }
        //calculating abs ratio 0 to 1
        double zeroToOne = (Math.abs(targetDegrees - angle)) / _degreeRange;
        if (zeroToOne > 1) {
            zeroToOne = 1;
        } else if (zeroToOne < 0) {
            zeroToOne = 0;
        }
        //squaring to drop off input faster as approaching zero
        zeroToOne = zeroToOne * zeroToOne;
        if(angle < 0){
            return -zeroToOne;
        } else {
            return zeroToOne;
        }
    }
}
