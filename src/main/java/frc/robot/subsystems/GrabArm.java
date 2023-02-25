package frc.robot.subsystems;

import java.util.Hashtable;

import javax.naming.LimitExceededException;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabArm extends SubsystemBase {
    private final Solenoid _grabberSolenoid = new Solenoid(14, PneumaticsModuleType.REVPH, 0);
    private GrabArmRotations _grabArmRotation = GrabArmRotations.Stowed;
    private GrabArmExtensions _grabArmExtension = GrabArmExtensions.Stowed;
    private final Hashtable<GrabArmRotations,GrabArmExtensions> _maxExtensions = new Hashtable<GrabArm.GrabArmRotations,GrabArm.GrabArmExtensions>();

    public GrabArm() {
        super();
        _maxExtensions.put(GrabArmRotations.Stowed, GrabArmExtensions.Stowed);
        _maxExtensions.put(GrabArmRotations.Substation, GrabArmExtensions.Substation);
        _maxExtensions.put(GrabArmRotations.TopCone, GrabArmExtensions.Top);
        _maxExtensions.put(GrabArmRotations.TopCube, GrabArmExtensions.Top);
        _maxExtensions.put(GrabArmRotations.MidCone, GrabArmExtensions.Mid);
        _maxExtensions.put(GrabArmRotations.MidCube, GrabArmExtensions.Mid);
        _maxExtensions.put(GrabArmRotations.Floor, GrabArmExtensions.Floor);
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
