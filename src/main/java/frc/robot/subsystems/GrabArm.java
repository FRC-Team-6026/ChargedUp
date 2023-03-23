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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabArm extends SubsystemBase {
    private final Solenoid _grabberSolenoid = new Solenoid(14, PneumaticsModuleType.REVPH, 1);
    private GrabArmPositions _grabArmPositions = GrabArmPositions.Substation;
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
    private double _compensationRotation = 0;
    private double _compensationExtension = 0;
    private boolean _isConeMode = false;
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

        this.setDefaultCommand(new FunctionalCommand(() -> {/*do nothing on init*/},
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

        SmartDashboard.putNumber("rotation velocity", _rotationEncoder.getVelocity());
        SmartDashboard.putNumber("extension velocity", _extensionEncoder.getVelocity());

        SmartDashboard.putString("SelectionRotation", _grabArmPositions.name());

        if (_rotationLimitSwitch.isPressed()) {
            _rotationEncoder.setPosition(0);
            _stationaryRotation = 0;
        }

        if (_extensionLimitSwitch.isPressed()) {
            _extensionEncoder.setPosition(0);
            _stationaryExtension = 0;
        }
    }

    public void closeGrabber() {
        _grabberSolenoid.set(false);
    }

    public void openGrabber() {
        _grabberSolenoid.set(true);
    }

    public CommandBase goToStowedPosition(){
        return runOnce(() -> { _stationaryExtension = 0; })
            .andThen(Commands.waitUntil(() -> _extensionEncoder.getPosition() <= _stationaryExtension + 1))
            .andThen(() -> {if (_stationaryExtension == 0) _stationaryRotation = 0;});
    }

    public void cycleNext() {
        _grabArmPositions = _grabArmPositions.next();
    }

    public void cyclePrevious() {
        _grabArmPositions = _grabArmPositions.previous();
    }

    public void goToPosition() {
        _stationaryRotation = _grabArmPositions.rotation;
        _stationaryExtension = _grabArmPositions.extension;
    }

    private void manualControls() {

        var rotationRatio = MathUtil.applyDeadband(_rotationSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
        var extensionRatio = MathUtil.applyDeadband(_extensionSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
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
            _isStationaryRotation=false;
        }

        disengageServo();

        //Comensation Calculations
        double centerOfGrav = (7.5+(0.254*_extensionEncoder.getPosition()));
        double cosineCompensation = Math.cos(Math.toRadians(_rotationEncoder.getPosition() - Constants.GrabArm.rotationOffsetinDegrees));
        double inLbTorque = (5 * centerOfGrav * cosineCompensation);
        double newtonMeterTorque = inLbTorque / 8.8507457673787;
        double motorOutput = newtonMeterTorque / Constants.GrabArm.rotationGearRatio;
        _compensationRotation = motorOutput / Constants.GrabArm.rotationStallTorque; // output / stall torque

        //Compensation Addition for Cone Mode
        if (_isConeMode == true){
            double conePivotLength = Constants.GrabArm.baseArmLength - 3 + _extensionEncoder.getPosition(); //Base Arm - into claw + arm extension ~= Cone Location relative to pivot
            double inLbTorqueCone = Constants.GrabArm.coneWeightLb * conePivotLength * cosineCompensation;
            double newtonMeterTorqueCone = inLbTorqueCone / 8.8507457673787;
            double motorOutputCone = newtonMeterTorqueCone / Constants.GrabArm.rotationGearRatio;
            _compensationRotation = _compensationRotation + (motorOutputCone / Constants.GrabArm.rotationStallTorque);
        }

        if (!_isStationaryRotation) {           
            if(extensionHeight > Constants.GrabArm.maxExtensionHeight){
                _stationaryExtension = Constants.GrabArm.maxExtensionHeight;
            }
            _targetRotation = _targetRotation + rotationSpeedDps;
            //Position Setting
            _rotationController.setReference(_targetRotation, ControlType.kPosition, 0, _compensationRotation, ArbFFUnits.kPercentOut);
            
        } else {
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            _rotationController.setReference(_stationaryRotation, ControlType.kSmartMotion,1, _compensationRotation, ArbFFUnits.kPercentOut);
        }

        //set the stationary inches when the extension comes to a stop.
        if (extensionRatio == 0 && !_isStationaryExtension) {
            _isStationaryExtension = true;
            _stationaryExtension = _extensionEncoder.getPosition();
        } else if (extensionRatio != 0) {
            _isStationaryExtension=false;
        } 

        //Compensation Calculations
        double armAngle = _rotationEncoder.getPosition() - Constants.GrabArm.rotationOffsetinDegrees;
        double tensionLB1stStage = (Constants.GrabArm.firstStageTension - Constants.GrabArm.firstStageAprxWeight * Math.sin(Math.toRadians(armAngle))); //Spring force - (Weight * sin (armAngle))
        //double tensionLB2ndStage = (Constants.GrabArm.secondStageTension - Constants.GrabArm.secondStageAprxWeight * Math.sin(Math.toRadians(armAngle))); //Spring force - (Weight * sin (armAngle))
        extensionStageCompensationCalculations(tensionLB1stStage);        

        if (!_isStationaryExtension) {
            //Extension Velocity Setting
            _targetExtension = _targetExtension + extensionIps;
            _extensionController.setReference(_targetExtension, ControlType.kPosition, 0, _compensationExtension, ArbFFUnits.kPercentOut);
        } else {
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            _extensionController.setReference(_stationaryExtension, ControlType.kPosition,1, _compensationExtension, ArbFFUnits.kPercentOut);
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
        _rotationController.setOutputRange(Constants.GrabArm.rotationMin, Constants.GrabArm.rotationMax);
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
        _extensionController.setP(Constants.GrabArm.extensionKP);
        _extensionController.setI(Constants.GrabArm.extensionKI);
        _extensionController.setD(Constants.GrabArm.extensionKD);
        _extensionController.setFF(Constants.GrabArm.extensionKFF);
        _extensionController.setOutputRange(Constants.GrabArm.extensionMin, Constants.GrabArm.extensionMax);
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

    private void disengageServo(){
        _ratchetServo.setAngle(90);
    }

    public void engageServo(){
        _ratchetServo.setAngle(30);
    }

    private void extensionStageCompensationCalculations(double tension){
        double inLBTorque = Constants.GrabArm.spoolRadius * tension;
        double newtonMeterTorque = inLBTorque / 8.8507457673787;
        double motorOutput = newtonMeterTorque / Constants.GrabArm.extensionGearRatio;
        _compensationExtension = - motorOutput / Constants.GrabArm.extensionStallTorque; // output / stall torque

    }

    public void isCone(){
        _isConeMode = !_isConeMode;
    }

    public void setRotationTargetTest(){
        _stationaryRotation = 30;
    }

    public enum GrabArmPositions {
        Substation (0,0) {
            @Override
            public GrabArmPositions previous() {
                return this;
            };
        },
        TopCone(180,0),
        TopCube(180,0),
        MidCone(180,0),
        MidCube(180,0),
        Floor(180,0) {
            @Override
            public GrabArmPositions next() {
                return this;
            };
        };
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
}
