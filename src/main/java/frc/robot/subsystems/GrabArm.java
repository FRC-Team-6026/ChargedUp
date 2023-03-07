package frc.robot.subsystems;

import java.util.Hashtable;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabArm extends SubsystemBase {
    private final Solenoid _grabberSolenoid = new Solenoid(14, PneumaticsModuleType.REVPH, 1);
    private GrabArmRotations _grabArmRotation = GrabArmRotations.Stowed;
    private GrabArmExtensions _grabArmExtension = GrabArmExtensions.Stowed;
    private final Hashtable<GrabArmRotations,GrabArmExtensions> _maxExtensions = new Hashtable<GrabArm.GrabArmRotations,GrabArm.GrabArmExtensions>();
    private final CANSparkMax _rotationMotor = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax _extensionMotor = new CANSparkMax(16, MotorType.kBrushless);
    private final RelativeEncoder _rotationEncoder;
    private final RelativeEncoder _extensionEncoder;
    private final SparkMaxPIDController _rotationController;
    private final SparkMaxPIDController _extensionController;
    private final SparkMaxLimitSwitch _rotationLimitSwitch;
    private final SparkMaxLimitSwitch _extensionLimitSwitch;
    private final Servo _ratchetServo = new Servo(9);
    private final DoubleSupplier _rotationSupplier;
    private final DoubleSupplier _extensionSupplier;

    private boolean _isStationaryRotation = true;
    private boolean _isStationaryExtension = true;
    private double _stationaryRotation = 0;
    private double _stationaryExtension = 0;
    private double _targettedRotation = 0;
    private double _targettedExtension = 0;
    private double _compensationRotation = 0;
    private double _compensationExtension = .1;

    public GrabArm(DoubleSupplier rotationSupplier, DoubleSupplier extensionSupplier) {
        super();

        _rotationSupplier = rotationSupplier;
        _extensionSupplier = extensionSupplier;

        _maxExtensions.put(GrabArmRotations.Stowed, GrabArmExtensions.Stowed);
        _maxExtensions.put(GrabArmRotations.Substation, GrabArmExtensions.Substation);
        _maxExtensions.put(GrabArmRotations.TopCone, GrabArmExtensions.Top);
        _maxExtensions.put(GrabArmRotations.TopCube, GrabArmExtensions.Top);
        _maxExtensions.put(GrabArmRotations.MidCone, GrabArmExtensions.Mid);
        _maxExtensions.put(GrabArmRotations.MidCube, GrabArmExtensions.Mid);
        _maxExtensions.put(GrabArmRotations.Floor, GrabArmExtensions.Floor);

        _rotationEncoder = _rotationMotor.getEncoder();
        _extensionEncoder = _extensionMotor.getEncoder();
        _rotationController = _rotationMotor.getPIDController();
        _extensionController = _extensionMotor.getPIDController();
        _rotationLimitSwitch = _rotationMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _extensionLimitSwitch = _extensionMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        configRotationMotor();
        configExtensionMotor();

        _stationaryRotation = _rotationEncoder.getPosition();
        _targettedRotation = _rotationEncoder.getPosition();
        _stationaryExtension = _rotationEncoder.getPosition();
        _targettedExtension = _extensionEncoder.getPosition();

        this.setDefaultCommand(new FunctionalCommand(() -> {/*do nothing on init*/},
            // do arcade drive by default
            () -> {manualControls();},
            //when interrupted set PID controls to voltage and default to 0 to stop
            interrupted ->
            {
            _rotationController.setReference(0, ControlType.kVoltage);
            _extensionController.setReference(0, ControlType.kVoltage);
            },
            //never end
            () -> {return false;},
            this));
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("rotation value", _rotationSupplier.getAsDouble());
        SmartDashboard.putNumber("extension value", _extensionSupplier.getAsDouble());

        SmartDashboard.putNumber("rotation angle", _rotationEncoder.getPosition());
        SmartDashboard.putNumber("extension inches", _extensionEncoder.getPosition());

        SmartDashboard.putBoolean("rotationswitchenable", _rotationLimitSwitch.isLimitSwitchEnabled());
        SmartDashboard.putBoolean("rotationswitchstatus", _rotationLimitSwitch.isPressed());
        SmartDashboard.putBoolean("extensionswitchenable", _extensionLimitSwitch.isLimitSwitchEnabled());
        SmartDashboard.putBoolean("extensionswitchstatus", _extensionLimitSwitch.isPressed());

        if (_rotationLimitSwitch.isPressed()) {
            _rotationEncoder.setPosition(0);
        }

        if (_extensionLimitSwitch.isPressed()) {
            _extensionEncoder.setPosition(0);
        }
    }

    public void closeGrabber() {
        _grabberSolenoid.set(true);
    }

    public void openGrabber() {
        _grabberSolenoid.set(false);
    }

    public void goToStowedPosition() {
        _grabArmRotation = GrabArmRotations.Stowed;
        _grabArmExtension = GrabArmExtensions.Stowed;
        goToCurrentPositions();
    }

    public void goToNextRotation() {
        _grabArmRotation = _grabArmRotation.next();
        limitExstensionToMax();
        goToCurrentPositions();
    }

    public void goToPreviousRotation() {
        _grabArmRotation = _grabArmRotation.previous();
        limitExstensionToMax();
        goToCurrentPositions();
    }

    public void goToNextExtension() {
        _grabArmExtension = _grabArmExtension.next();
        limitExstensionToMax();
        goToCurrentPositions();
    }

    public void goToPreviousExtension() {
        _grabArmExtension = _grabArmExtension.previous();
        limitExstensionToMax();
        goToCurrentPositions();
    }

    public void goToPosition(GrabArmExtensions extension, GrabArmRotations rotation) {
        //to be implemented
        //drive extension motor and rotation motor by position to the positions specified
    }

    private void manualControls() {
        var rotationRatio = MathUtil.applyDeadband(_rotationSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
        var extensionRatio = MathUtil.applyDeadband(_extensionSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
        //cubing inputs to give better control over the low range.
        rotationRatio = rotationRatio * rotationRatio * rotationRatio;
        extensionRatio = extensionRatio * extensionRatio * extensionRatio;      
        var rotationSpeedDps = rotationRatio * Constants.GrabArm.maxRotationHz;
        var extensionIps = extensionRatio * Constants.GrabArm.maxIpsHz;
        if (extensionRatio != 0) {
            _ratchetServo.setAngle(80);
        } else {
            _ratchetServo.setAngle(30);
        }

        //set the stationary rotation when the arm comes to a stop.
        if (rotationRatio == 0 && !_isStationaryRotation) {
            _isStationaryRotation = true;
            _stationaryRotation = _rotationEncoder.getPosition();
            _targettedRotation = _stationaryRotation;
        } else if (rotationRatio != 0) {
            _isStationaryRotation=false;
        }

        if (!_isStationaryRotation) {
            //Comensation Calculations
            double centerOfGrav = (7.5+(0.254*_extensionEncoder.getPosition()));
            double inLbTorque = (10 * centerOfGrav * Math.cos(Math.abs(Math.toRadians(_rotationEncoder.getPosition()-39))));
            double newtonMeterTorque = inLbTorque / 8.8507457673787;
            double motorOutput = newtonMeterTorque / Constants.GrabArm.rotationGearRatio;
            _compensationRotation = motorOutput / 2.6;

            //Rotation Targetting
            _targettedRotation = _targettedRotation + rotationSpeedDps;
            if(_targettedRotation > Constants.GrabArm.rotationForwardSoftLimitDegrees){
                _targettedRotation = Constants.GrabArm.rotationForwardSoftLimitDegrees;
            }

            //Position Setting
            SmartDashboard.putNumber("targetRotation", _targettedRotation);
            _rotationController.setReference(_targettedRotation, ControlType.kPosition, 0, _compensationRotation, ArbFFUnits.kPercentOut);
            
        } else {
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            _rotationController.setReference(_stationaryRotation, ControlType.kPosition,0, _compensationExtension, ArbFFUnits.kPercentOut);
        }

        //set the stationary inches when the extension comes to a stop.
        if (extensionRatio == 0 && !_isStationaryExtension) {
            _isStationaryExtension = true;
            _stationaryExtension = _rotationEncoder.getPosition();
            _targettedExtension = _stationaryExtension;
        } else if (rotationRatio != 0) {
            _isStationaryRotation=false;
        }    

        if (!_isStationaryExtension) {
            //Extension Targetting
            _targettedExtension = _targettedExtension + extensionIps;
            if(_targettedExtension > Constants.GrabArm.extensionForwardSoftLimitInches){
                _targettedExtension = Constants.GrabArm.extensionForwardSoftLimitInches;
            }
            
            //Extension setting
            SmartDashboard.putNumber("targetExtension", _targettedExtension);
            _extensionController.setReference(_targettedExtension, ControlType.kPosition, 0, _compensationExtension, ArbFFUnits.kPercentOut);
            
        } else {
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            _extensionController.setReference(_stationaryExtension, ControlType.kPosition,0, _compensationExtension, ArbFFUnits.kPercentOut);
        }
    }

    private void configRotationMotor() {
        _rotationMotor.restoreFactoryDefaults();
        _rotationMotor.setSmartCurrentLimit(Constants.GrabArm.rotationContinuousCurrentLimit);
        _rotationMotor.setInverted(Constants.GrabArm.rotationInvert);
        _rotationMotor.setIdleMode(Constants.GrabArm.rotationNeutralMode);
        _rotationEncoder.setPositionConversionFactor(Constants.GrabArm.rotationConversionFactor);
        _rotationEncoder.setVelocityConversionFactor(Constants.GrabArm.rotationVelocityConversionFactorDps);
        _rotationController.setP(Constants.GrabArm.rotationKP);
        _rotationController.setI(Constants.GrabArm.rotationKI);
        _rotationController.setD(Constants.GrabArm.rotationKD);
        _rotationController.setFF(Constants.GrabArm.rotationKFF);
        _rotationMotor.enableVoltageCompensation(Constants.GrabArm.voltageComp);
        _rotationMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.GrabArm.rotationForwardSoftLimitDegrees);
        _rotationController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        _rotationController.setSmartMotionMaxAccel(Constants.GrabArm.maxRotationAccDps, 0);
        _rotationController.setSmartMotionMaxVelocity(Constants.GrabArm.maxRotationDps, 0);
        _rotationMotor.burnFlash();
    }

    private void configExtensionMotor() {
        _extensionMotor.restoreFactoryDefaults();
        _extensionMotor.setSmartCurrentLimit(Constants.GrabArm.extensionContinuousCurrentLimit);
        _extensionMotor.setInverted(Constants.GrabArm.extensionInvert);
        _extensionMotor.setIdleMode(Constants.GrabArm.extensionNeutralMode);
        _extensionEncoder.setPositionConversionFactor(Constants.GrabArm.extensionConversionFactorInches);
        _extensionEncoder.setVelocityConversionFactor(Constants.GrabArm.extensionVelocityConversionFactorIps);
        _extensionController.setP(Constants.GrabArm.extensionKP);
        _extensionController.setI(Constants.GrabArm.extensionKI);
        _extensionController.setD(Constants.GrabArm.extensionKD);
        _extensionController.setFF(Constants.GrabArm.extensionKFF);
        _extensionMotor.enableVoltageCompensation(Constants.GrabArm.voltageComp);
        _extensionMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.GrabArm.extensionForwardSoftLimitInches);
        _extensionController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        _extensionController.setSmartMotionMaxAccel(Constants.GrabArm.maxIpsAcc, 0);
        _extensionController.setSmartMotionMaxVelocity(Constants.GrabArm.maxIps, 0);
        _extensionMotor.burnFlash();
    }

    private void goToCurrentPositions() {
        goToPosition(_grabArmExtension, _grabArmRotation);
    }

    private void limitExstensionToMax()
    {
        var maxExtension = _maxExtensions.get(_grabArmRotation);
        if (maxExtension.ordinal() < _grabArmExtension.ordinal()) {
            _grabArmExtension = maxExtension;
        }
    }

    public enum GrabArmExtensions {
        Stowed {
            @Override
            public GrabArmExtensions previous() {
                return this;
            };
        },
        Substation,
        Floor,
        Mid,
        Top {
            @Override
            public GrabArmExtensions next() {
                return this;
            };
        };

        public GrabArmExtensions next() {
            // No bounds checking required here, because the last instance overrides
            return values()[ordinal() + 1];
        }
        public GrabArmExtensions previous() {
            // No bounds checking required here, because the first instance overrides
            return values()[ordinal() - 1];
        }
    }

    public enum GrabArmRotations {
        Stowed {
            @Override
            public GrabArmRotations previous() {
                return this;
            };
        },
        Substation,
        TopCone,
        TopCube,
        MidCone,
        MidCube,
        Floor {
            @Override
            public GrabArmRotations next() {
                return this;
            };
        };

        public GrabArmRotations next() {
            // No bounds checking required here, because the last instance overrides
            return values()[ordinal() + 1];
        }
        public GrabArmRotations previous() {
            // No bounds checking required here, because the first instance overrides
            return values()[ordinal() - 1];
        }
    }
}
