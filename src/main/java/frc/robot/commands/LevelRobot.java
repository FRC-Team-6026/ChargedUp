package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class LevelRobot extends CommandBase{
    private double _tolerance = Constants.Swerve.maxPitchDegrees;
    private double _maxPitch = 20;
    private double _pitchRange = _maxPitch - _tolerance;
    private double _maxVelocityMs = Constants.Swerve.levelingMaxVelocityMs;

    private final Timer _timer = new Timer();

    private final Swerve _swerve;
    private final AHRS _gyro;
    
    public LevelRobot(Swerve swerve, AHRS gyro) {
        super();
        _swerve = swerve;
        _gyro = gyro;
        addRequirements(_swerve);
    }

    @Override
    public void initialize(){
        _timer.start();
    }

    @Override
    public void execute(){
        var pitch = getPitchRatio();
        _swerve.drive(new Translation2d(pitch,0).times(_maxVelocityMs), 0, true, true);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(_gyro.getPitch()) < _tolerance && _timer.advanceIfElapsed(2)){
            return true;
        } else if(_gyro.getPitch() > _tolerance){
            _timer.restart();
        }
        return false;
    }

    private double getPitchRatio(){
        double pitch = _gyro.getPitch();
        if(MathUtil.applyDeadband(pitch, _tolerance) == 0){
            return 0.0;
        }
        //calculating abs ratio 0 to 1
        double zeroToOne = (Math.abs(pitch) - _tolerance) / _pitchRange;
        if (zeroToOne > 1) {
            zeroToOne = 1;
        } else if (zeroToOne < 0) {
            zeroToOne = 0;
        }
        //squaring to drop off input faster as approaching zero
        zeroToOne = zeroToOne * zeroToOne;
        if(pitch < 0){
            return -zeroToOne;
        } else {
            return zeroToOne;
        }

    }
}
