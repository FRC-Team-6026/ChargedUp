package frc.robot.commands;

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

    private final Timer _timer = new Timer();

    private final Swerve _swerve;
    
    public LevelRobot(Swerve swerve) {
        super();
        _swerve = swerve;
        addRequirements(_swerve);
    }

    @Override
    public void initialize(){
        _timer.start();
    }

    @Override
    public void execute(){
        var pitch = getPitchRatio();
        _swerve.drive(new Translation2d(pitch,0), 0, true, true);
    }

    @Override
    public boolean isFinished(){
        if(Math.abs(_swerve.getPitch()) < _tolerance && _timer.advanceIfElapsed(2)){
            return true;
        } else if(_swerve.getPitch() > _tolerance){
            _timer.restart();
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        _swerve.xPattern();
    }

    private double getPitchRatio(){
        double pitch = _swerve.getPitch();
        if(MathUtil.applyDeadband(pitch, _tolerance) == 0){
            return 0.0;
        }
        //calculating abs ratio 0 to 1 then multiplied by speed
        double zeroToOne = ((Math.abs(pitch) - _tolerance) / _pitchRange) * Constants.Swerve.levelingMaxVelocityMs;
        if (zeroToOne > 1) {
            zeroToOne = 1;
        } else if (zeroToOne < 0) {
            zeroToOne = 0;
        }

        if(pitch < 0){
            return -zeroToOne;
        } else {
            return zeroToOne;
        }

    }
}
