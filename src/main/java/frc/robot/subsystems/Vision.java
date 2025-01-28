package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private double _posX; // "Pose X"
    private double _posY; // "Pose Y"
    private double _rotation; // "Tag Rotation"
    private int _tagID; // "Widest Tag ID"
    private boolean _tagPresent; // "AprilTag Presence"

    private BooleanSubscriber _tagPresenceSub;
    private DoubleSubscriber _rotationSub;
    private DoubleSubscriber _posXSub;
    private DoubleSubscriber _posYSub;
    private IntegerSubscriber _tagIDSub;

    public Vision() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("AprilTag Vision");
        _tagPresenceSub = table.getBooleanTopic("AprilTag Presence").subscribe(false);
        _rotationSub = table.getDoubleTopic("Tag Rotation").subscribe(0);
        _posXSub = table.getDoubleTopic("Pose X").subscribe(0);
        _posYSub = table.getDoubleTopic("Pose Y").subscribe(0);
        _tagIDSub = table.getIntegerTopic("Widest Tag ID").subscribe(0);
    }

    public Transform2d getCameraToAprilTag() {
        _tagPresent = _tagPresenceSub.get();
        if(_tagPresent == true) {
            _rotation = _rotationSub.get();
            _posX = _posXSub.get();
            _posY = _posYSub.get();
            return new Transform2d(_posX, _posY, new Rotation2d(_rotation));
        }
        return null;
    }

    public int getBestAprilTagID () {
        _tagID =  _tagIDSub.get();
        return _tagID;
    }
}