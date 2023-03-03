package frc.robot.subsystems;

import java.util.Hashtable;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private final Servo _ratchetServo = new Servo(9);
    private final DoubleSupplier _rotationSupplier;
    private final DoubleSupplier _extensionSupplier;
    private final SlewRateLimiter _rotationLimiter = new SlewRateLimiter(30);
    private final SlewRateLimiter _extensionLimiter = new SlewRateLimiter(10);

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

        configRotationMotor();
        configExtensionMotor();

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
        //var rotationSpeedDps = _rotationLimiter.calculate(rotationRatio * Constants.GrabArm.maxRotationDps);
        //var extensionIps = _extensionLimiter.calculate(extensionRatio * Constants.GrabArm.maxIps);
        rotationRatio = rotationRatio * rotationRatio * rotationRatio * 0.5;
        extensionRatio = extensionRatio * extensionRatio * extensionRatio * 0.5;      
        if (extensionRatio > 0) {
            _ratchetServo.setAngle(80);
        } else {
            _ratchetServo.setAngle(0);
        }

        _rotationController.setReference(rotationRatio, ControlType.kDutyCycle);
        _extensionController.setReference(extensionRatio, ControlType.kDutyCycle);
    }

    private void configRotationMotor() {
        _rotationMotor.restoreFactoryDefaults();
        _rotationMotor.setSmartCurrentLimit(Constants.GrabArm.rotationContinuousCurrentLimit);
        _rotationMotor.setInverted(Constants.GrabArm.rotationInvert);
        _rotationMotor.setIdleMode(Constants.GrabArm.rotationNeutralMode);
        _rotationEncoder.setPositionConversionFactor(Constants.GrabArm.rotationConversionFactor);
        _rotationController.setP(Constants.GrabArm.rotationKP);
        _rotationController.setI(Constants.GrabArm.rotationKI);
        _rotationController.setD(Constants.GrabArm.rotationKD);
        _rotationController.setFF(Constants.GrabArm.rotationKFF);
        _rotationMotor.enableVoltageCompensation(Constants.GrabArm.voltageComp);
        _rotationMotor.burnFlash();
    }

    private void configExtensionMotor() {
        _extensionMotor.restoreFactoryDefaults();
        _extensionMotor.setSmartCurrentLimit(Constants.GrabArm.extensionContinuousCurrentLimit);
        _extensionMotor.setInverted(Constants.GrabArm.extensionInvert);
        _extensionMotor.setIdleMode(Constants.GrabArm.extensionNeutralMode);
        _extensionEncoder.setPositionConversionFactor(Constants.GrabArm.extensionConversionFactorInches);
        _extensionController.setP(Constants.GrabArm.extensionKP);
        _extensionController.setI(Constants.GrabArm.extensionKI);
        _extensionController.setD(Constants.GrabArm.extensionKD);
        _extensionController.setFF(Constants.GrabArm.extensionKFF);
        _extensionMotor.enableVoltageCompensation(Constants.GrabArm.voltageComp);
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
