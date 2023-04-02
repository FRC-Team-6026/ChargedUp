package frc.robot.subsystems;

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
    private GrabArmPositions _grabArmPositions = GrabArmPositions.Substation;


    private boolean _isStationaryRotation = true;
    private boolean _isStationaryExtension = true;
    private double _stationaryRotation = 0;
    private double _stationaryExtension = 0;
    private double _desiredRotation = 0;
    private double _desiredExtension = 0;
    private double _compensationRotation = 0;
    private double _compensationExtension = 0;
    private double _targetExtension = 0;
    private double _targetRotation = 0;


    public GrabArm(DoubleSupplier rotationSupplier, DoubleSupplier extensionSupplier) {
        super();

        _rotationSupplier = rotationSupplier;
        _extensionSupplier = extensionSupplier;

        _rotationEncoder = _rotationMotor.getEncoder();
        _extensionEncoder = _extensionMotor.getEncoder();
        _rotationController = _rotationMotor.getPIDController();
        _extensionController = _extensionMotor.getPIDController();
        _rotationLimitSwitch = _rotationMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        _extensionLimitSwitch = _extensionMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        configRotationMotor();
        configExtensionMotor();

        _stationaryRotation = _rotationEncoder.getPosition();
        _stationaryExtension = _extensionEncoder.getPosition();

        this.setDefaultCommand(controlLoop());
    }

    private FunctionalCommand controlLoop(){
    return new FunctionalCommand(() -> {/*do nothing on init*/},
    // do arcade drive by default
    () -> {manualControls();},
    //when interrupted set PID controls to voltage and default to 0 to stop
    interrupted ->
    {
    engageServo();
    _rotationController.setReference(0, ControlType.kVoltage);
    _extensionController.setReference(0, ControlType.kVoltage);
    },
    //never end
    () -> {return false;},
    this);
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

        SmartDashboard.putNumber("rotation velocity", _rotationEncoder.getVelocity());
        SmartDashboard.putNumber("extension velocity", _extensionEncoder.getVelocity());

        SmartDashboard.putNumber("target rotation", _targetRotation);
        SmartDashboard.putNumber("target extension", _targetExtension);
        SmartDashboard.putNumber("stationary rotation", _stationaryRotation);
        SmartDashboard.putNumber("stationary extension", _stationaryExtension);

        SmartDashboard.putString("SelectionRotation", _grabArmPositions.name());

        if (_rotationLimitSwitch.isPressed()) {
            _rotationEncoder.setPosition(0);
            _stationaryRotation = 0;
            _targetRotation = 0;
        }

        if (_extensionLimitSwitch.isPressed()) {
            _extensionEncoder.setPosition(0);
            _stationaryExtension = 0;
            _targetExtension = 0;
        }
    }

    public void closeGrabber() {
        _grabberSolenoid.set(false);
    }

    public void openGrabber() {
        _grabberSolenoid.set(true);
    }

    public void cycleNext() {
        _grabArmPositions = _grabArmPositions.next();
    }

    public void cyclePrevious() {
        _grabArmPositions = _grabArmPositions.previous();
    }

    private void manualControls() {
        double rotationRatio;
        double extensionRatio;
        rotationRatio = MathUtil.applyDeadband(_rotationSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
        extensionRatio = MathUtil.applyDeadband(_extensionSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);

        //cubing inputs to give better control over the low range.
        rotationRatio = rotationRatio * rotationRatio * rotationRatio;
        extensionRatio = extensionRatio * extensionRatio * extensionRatio;
        
        var rotationSpeedDps = rotationRatio * Constants.GrabArm.maxRotationExecution;
        var extensionIps = extensionRatio * Constants.GrabArm.maxIpsExecution;

        double extensionHeight = (_extensionEncoder.getPosition() + Constants.GrabArm.baseArmLength) * Math.sin(_rotationEncoder.getPosition() - Constants.GrabArm.rotationOffsetinDegrees);

        //set the stationary rotation when the arm comes to a stop.
        if (rotationRatio == 0 && !_isStationaryRotation) {
            _isStationaryRotation = true;
            _stationaryRotation = _rotationEncoder.getPosition();
            _targetRotation = _stationaryRotation;
        } else if (rotationRatio != 0) {
            if(_isStationaryRotation){
                _isStationaryRotation = !_isStationaryRotation;
                _targetRotation = _rotationEncoder.getPosition();
            }
        }

        if (!_isStationaryRotation) {           
            _targetRotation = _targetRotation + rotationSpeedDps;
            //Position Setting
            _rotationController.setReference(_targetRotation, ControlType.kPosition, 0, _compensationRotation, ArbFFUnits.kPercentOut);
            
        } else {
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            stationaryRotation();
        }

        //set the stationary inches when the extension comes to a stop.
        if (extensionRatio == 0 && !_isStationaryExtension) {
            _isStationaryExtension = true;
            _stationaryExtension = _extensionEncoder.getPosition();
            _targetExtension = _stationaryExtension;
        } else if (extensionRatio != 0) {
            if(_isStationaryExtension){
                _isStationaryExtension = !_isStationaryExtension;
                _targetExtension = _extensionEncoder.getPosition();
            }
        } 

        if(extensionHeight > Constants.GrabArm.maxExtensionHeight){
            _stationaryExtension = Constants.GrabArm.maxExtensionHeight;
        }

        if( !_isStationaryExtension || _stationaryExtension - 0.1 > _extensionEncoder.getPosition()){
            disengageServo();
        } else {
            engageServo();
        }

        if (!_isStationaryExtension) {
            _targetExtension = _targetExtension + extensionIps;
            if ( _targetExtension < _extensionEncoder.getPosition() && _extensionEncoder.getPosition() < 10.3){
                    _compensationExtension = _compensationExtension * Constants.GrabArm.tensionLesseningFactor;
            }
            else if ( _targetExtension > _extensionEncoder.getPosition() && _extensionEncoder.getPosition() > 10){
                    _compensationExtension = _compensationExtension * Constants.GrabArm.tensionLesseningFactor;
            }
            _extensionController.setReference(_targetExtension, ControlType.kPosition, 0, _compensationExtension, ArbFFUnits.kPercentOut); // Movement requires a lesser feed foward/ Not calculated
        } else {
            if ( _stationaryExtension < _extensionEncoder.getPosition() && _extensionEncoder.getPosition() < 10.3){
                _compensationExtension = _compensationExtension * Constants.GrabArm.tensionLesseningFactor;
            }
            else if ( _stationaryExtension > _extensionEncoder.getPosition() && _extensionEncoder.getPosition() > 10){
                _compensationExtension = _compensationExtension * Constants.GrabArm.tensionLesseningFactor;
            }
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            stationaryExtension();
        }
    }

    private void configRotationMotor() {
        _rotationMotor.restoreFactoryDefaults();
        _rotationMotor.setSmartCurrentLimit(Constants.GrabArm.rotationContinuousCurrentLimit);
        _rotationMotor.setInverted(Constants.GrabArm.rotationInvert);
        _rotationMotor.setIdleMode(Constants.GrabArm.rotationNeutralMode);
        _rotationEncoder.setPositionConversionFactor(Constants.GrabArm.rotationConversionFactor);
        _rotationEncoder.setVelocityConversionFactor(Constants.GrabArm.rotationVelocityConversionFactorDps);
        _rotationController.setP(Constants.GrabArm.rotationKP,0);
        _rotationController.setI(Constants.GrabArm.rotationKI,0);
        _rotationController.setD(Constants.GrabArm.rotationKD,0);
        _rotationController.setP(Constants.GrabArm.rotationKPAuto,1);
        _rotationController.setI(Constants.GrabArm.rotationKIAuto,1);
        _rotationController.setD(Constants.GrabArm.rotationKDAuto,1);
        _rotationController.setFF(Constants.GrabArm.rotationKFF);
        _rotationController.setFF(Constants.GrabArm.rotationKFFAuto,1);
        _rotationController.setOutputRange(Constants.GrabArm.rotationMin, Constants.GrabArm.rotationMax,0);
        _rotationController.setOutputRange(Constants.GrabArm.rotationMin, Constants.GrabArm.rotationMax,1);
        _rotationMotor.enableVoltageCompensation(Constants.GrabArm.voltageComp);
        _rotationMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.GrabArm.rotationForwardSoftLimitDegrees);
        _rotationMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        _rotationController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        _rotationController.setSmartMotionMaxAccel(Constants.GrabArm.maxRotationAccDps, 0);
        _rotationController.setSmartMotionMaxVelocity(Constants.GrabArm.maxRotationDps, 0);
        _rotationController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);
        _rotationController.setSmartMotionMaxAccel(Constants.GrabArm.maxAutoPositionRotationDpsAcc, 1);
        _rotationController.setSmartMotionMaxVelocity(Constants.GrabArm.maxAutoPositionRotationDpsAcc, 1);

        _rotationMotor.burnFlash();
    }

    private void configExtensionMotor() {
        _extensionMotor.restoreFactoryDefaults();
        _extensionMotor.setSmartCurrentLimit(Constants.GrabArm.extensionContinuousCurrentLimit);
        _extensionMotor.setInverted(Constants.GrabArm.extensionInvert);
        _extensionMotor.setIdleMode(Constants.GrabArm.extensionNeutralMode);
        _extensionEncoder.setPositionConversionFactor(Constants.GrabArm.extensionConversionFactorInches);
        _extensionEncoder.setVelocityConversionFactor(Constants.GrabArm.extensionVelocityConversionFactorIps);
        _extensionController.setP(Constants.GrabArm.extensionKP,0);
        _extensionController.setI(Constants.GrabArm.extensionKI,0);
        _extensionController.setD(Constants.GrabArm.extensionKD,0);
        _extensionController.setFF(Constants.GrabArm.extensionKFF,0);
        _extensionController.setP(Constants.GrabArm.extensionKP,1);
        _extensionController.setI(Constants.GrabArm.extensionKI,1);
        _extensionController.setD(Constants.GrabArm.extensionKD,1);
        _extensionController.setFF(Constants.GrabArm.extensionKFF,1);
        _extensionController.setOutputRange(Constants.GrabArm.extensionMin, Constants.GrabArm.extensionMax,0);
        _extensionController.setOutputRange(Constants.GrabArm.extensionMin, Constants.GrabArm.extensionMax,1);
        _extensionMotor.enableVoltageCompensation(Constants.GrabArm.voltageComp);
        _extensionMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.GrabArm.extensionForwardSoftLimitInches);
        _extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        _extensionController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        _extensionController.setSmartMotionMaxAccel(Constants.GrabArm.maxIpsAcc, 0);
        _extensionController.setSmartMotionMaxVelocity(Constants.GrabArm.maxIps, 0);
        _extensionController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 1);
        _extensionController.setSmartMotionMaxAccel(Constants.GrabArm.maxAutoPositionMaxIpsAcc, 1);
        _extensionController.setSmartMotionMaxVelocity(Constants.GrabArm.maxAutoPositionMaxIps, 1);

        _extensionMotor.burnFlash();
    }

    public void disengageServo(){
        _ratchetServo.setAngle(90);
    }

    public void engageServo(){
        _ratchetServo.setAngle(30);
    }

    public void setRotationTargetTest(){
        _stationaryRotation = 30;
    }

    public enum GrabArmPositions {
        Substation (176,19) {
            @Override
            public GrabArmPositions previous() {
                return Floor;
            };
        },
        TopCone(172,19),
        TopCube(181.7,15.7),
        MidCone(178,3.9),
        MidCube(197,2),
        Floor(3,19) {
            @Override
            public GrabArmPositions next() {
                return Substation;
            };
        },
        Stow(35,0),
        TravelLimit(0,4);
        private final double rotation;
        private final double extension;
        GrabArmPositions(double rotation, double extension){
            this.rotation = rotation;
            this.extension = extension;
        }
        public GrabArmPositions next() {
            // No bounds checking required here, because the last instance overrides
            return values()[ordinal() + 1];
        }
        public GrabArmPositions previous() {
            // No bounds checking required here, because the first instance overrides
            return values()[ordinal() - 1];
        }
    }

    public void compensationComputation(){
        //Compensation Calculations
        double centerOfGrav = (7.5+(0.254*_extensionEncoder.getPosition()));
        double cosineCompensation = Math.cos(Math.toRadians(_rotationEncoder.getPosition() - Constants.GrabArm.rotationOffsetinDegrees));
        double inLbTorqueR = (5 * centerOfGrav * cosineCompensation);
        double newtonMeterTorqueR = inLbTorqueR / 8.8507457673787;
        double motorOutputR = newtonMeterTorqueR / Constants.GrabArm.rotationGearRatio;
        _compensationRotation = motorOutputR / Constants.GrabArm.rotationStallTorque; // output / stall torque

        double tensionFromSprings = (Constants.GrabArm.firstStageTension); //Spring force
        double inLBTorqueE = Constants.GrabArm.spoolRadius * tensionFromSprings;
        double newtonMeterTorqueE = inLBTorqueE / 8.8507457673787;
        double motorOutputE = newtonMeterTorqueE / Constants.GrabArm.extensionGearRatio;
        _compensationExtension = - (motorOutputE / Constants.GrabArm.extensionStallTorque) * 0.65; // output / stall torque
    }

    public void stationaryRotation(){
        _rotationController.setReference(_stationaryRotation, ControlType.kPosition,0, _compensationRotation, ArbFFUnits.kPercentOut);
    }

    public void stationaryExtension(){
        engageServo();
        _extensionController.setReference(_stationaryExtension, ControlType.kPosition,0, _compensationExtension, ArbFFUnits.kPercentOut);
    }

    public void setDesiredRotation(GrabArmPositions set){
        _desiredRotation = set.rotation;
    }

    public void setDesiredExtension(GrabArmPositions set){
        _desiredExtension = set.extension;
    }

    public void goToDesiredRotation(){
        _rotationController.setReference(_desiredRotation, ControlType.kSmartMotion,1, _compensationRotation, ArbFFUnits.kPercentOut);
    }

    public void goToDesiredExtension(){
        disengageServo();
        _extensionController.setReference(_desiredExtension, ControlType.kSmartMotion,1, _compensationExtension, ArbFFUnits.kPercentOut);
    }

    public void currentToStationary(){
        currentRotationToStationary();
        currentExtensionToStationary();
    }

    public void currentRotationToStationary(){
        _stationaryRotation = _rotationEncoder.getPosition();
    }

    public void currentExtensionToStationary(){
        _stationaryExtension = _extensionEncoder.getPosition();
    }

    public void desiredRotationToStationary(GrabArmPositions Rotation){
        _stationaryRotation = Rotation.rotation;
    }

    public void desiredExtensionToStationary(GrabArmPositions Extension){
        _stationaryExtension = Extension.extension;
    }

    public boolean checkRotation(GrabArmPositions pos){
        if(0 == MathUtil.applyDeadband(pos.rotation - _rotationEncoder.getPosition(), Constants.GrabArm.rotationPositionSettingToleranceDegrees)){
            return true;
        }
        return false;
    }

    public boolean checkExtension(GrabArmPositions pos){
        if(0 == MathUtil.applyDeadband(pos.extension - _extensionEncoder.getPosition(), Constants.GrabArm.extensionPositionSettingToleranceInches)){
            return true;
        }
        return false;
    }

    public boolean checkNeedToLimitTravelExtension(){
        if(_extensionEncoder.getPosition() > GrabArmPositions.TravelLimit.extension){
            return true;
        } else {
        return false;
        }
    }

}
