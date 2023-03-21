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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabArm extends SubsystemBase {
    private final Solenoid _grabberSolenoid = new Solenoid(14, PneumaticsModuleType.REVPH, 1);
    private GrabArmRotations _grabArmRotation = GrabArmRotations.Substation;
    private GrabArmExtensions _grabArmExtension = GrabArmExtensions.Substation;
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
    private double _compensationRotation = 0;
    private double _compensationExtension = 0;
    private boolean _isConeMode = false;

    public GrabArm(DoubleSupplier rotationSupplier, DoubleSupplier extensionSupplier) {
        super();

        _rotationSupplier = rotationSupplier;
        _extensionSupplier = extensionSupplier;

        _maxExtensions.put(GrabArmRotations.Substation, GrabArmExtensions.Substation);
        _maxExtensions.put(GrabArmRotations.TopCone, GrabArmExtensions.TopCone);
        _maxExtensions.put(GrabArmRotations.TopCube, GrabArmExtensions.TopCube);
        _maxExtensions.put(GrabArmRotations.MidCone, GrabArmExtensions.MidCone);
        _maxExtensions.put(GrabArmRotations.MidCube, GrabArmExtensions.MidCube);
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

        SmartDashboard.putString("SelectionRotation", _grabArmRotation.name());
        SmartDashboard.putString("SelectionExtension", _grabArmExtension.name());

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
            .andThen(Commands.waitUntil(() -> _extensionEncoder.getPosition() <= 1))
            .andThen(() -> _stationaryRotation = 0);
    }

    public void cycleNext() {
        _grabArmRotation = _grabArmRotation.next();
        _grabArmExtension = _grabArmExtension.next();
    }

    public void cyclePrevious() {
        _grabArmRotation = _grabArmRotation.previous();
        _grabArmExtension = _grabArmExtension.previous();
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
        _stationaryRotation = _grabArmRotation.rotation;
        _stationaryExtension = _grabArmExtension.extension;
    }

    private void manualControls() {

        var rotationRatio = MathUtil.applyDeadband(_rotationSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
        var extensionRatio = MathUtil.applyDeadband(_extensionSupplier.getAsDouble(), Constants.GrabArm.stickDeadband);
        //cubing inputs to give better control over the low range.
        rotationRatio = rotationRatio * rotationRatio * rotationRatio;
        extensionRatio = extensionRatio * extensionRatio * extensionRatio;      
        var rotationSpeedDps = rotationRatio * Constants.GrabArm.maxRotationDps;
        var extensionIps = extensionRatio * Constants.GrabArm.maxIps;

        double extensionHeight = (_extensionEncoder.getPosition() + Constants.GrabArm.baseArmLength) * Math.sin(_rotationEncoder.getPosition() - Constants.GrabArm.rotationOffsetinDegrees);

        //set the stationary rotation when the arm comes to a stop.
        if (rotationRatio == 0 && !_isStationaryRotation) {
            _isStationaryRotation = true;
            _stationaryRotation = _rotationEncoder.getPosition();
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

            //Position Setting
            _rotationController.setReference(rotationSpeedDps, ControlType.kSmartVelocity, 0, _compensationRotation, ArbFFUnits.kPercentOut);
            
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
            _extensionController.setReference(extensionIps, ControlType.kSmartMotion, 0, _compensationExtension, ArbFFUnits.kPercentOut);
        } else {
            //if the arm is stationary set the reference to position so that the arm doesn't drift over time
            _extensionController.setReference(_stationaryExtension, ControlType.kSmartMotion,1, _compensationExtension, ArbFFUnits.kPercentOut);
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

    private void goToCurrentPositions() {
        goToPosition(_grabArmExtension, _grabArmRotation);
    }

    private void disengageServo(){
        _ratchetServo.setAngle(90);
    }

    private void engageServo(){
        _ratchetServo.setAngle(30);
    }

    private void limitExstensionToMax()
    {
        var maxExtension = _maxExtensions.get(_grabArmRotation);
        if (maxExtension.ordinal() < _grabArmExtension.ordinal()) {
            _grabArmExtension = maxExtension;
        }
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

    public enum GrabArmExtensions {
        Substation(0) {
            @Override
            public GrabArmExtensions previous() {
                return this;
            };
        },TopCone(0),
        TopCube(0),
        MidCone(0),
        MidCube(0),
        Floor(0) {
            @Override
            public GrabArmExtensions next() {
                return this;
            };
        };

        private final double extension;
        GrabArmExtensions(double extension){
            this.extension = extension;
        }
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
        Substation (0) {
            @Override
            public GrabArmRotations previous() {
                return this;
            };
        },
        TopCone(180),
        TopCube(180),
        MidCone(180),
        MidCube(180),
        Floor(180) {
            @Override
            public GrabArmRotations next() {
                return this;
            };
        };
        private final double rotation;
        GrabArmRotations(double rotation){
            this.rotation = rotation;
        }
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
