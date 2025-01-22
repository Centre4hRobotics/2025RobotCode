package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private double _posX, _posY, _rotation;
    private int _tagID;
    private boolean _tagPresent;

    public Vision() {
        _posX = 0; // "Pose X"
        _posY = 0; // "Pose Y"
        _rotation = 0; // "Tag Rotation"
        _tagID = -1; // "Widest Tag ID"
        _tagPresent = false; // "AprilTag Presence"
    }

    public Transform2d getCameraToAprilTag() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("AprilTag Presence").getBoolean(_tagPresent);
        if(_tagPresent == true) {
            nt.getTable("AprilTag Vision").getEntry("Pose X").getDouble(_posX);
            nt.getTable("AprilTag Vision").getEntry("Pose Y").getDouble(_posY);
            nt.getTable("AprilTag Vision").getEntry("Tag Rotation").getDouble(_rotation);
            return new Transform2d(_posX, _posY, new Rotation2d(_rotation));
        }
        return new Transform2d();
    }

    public int getBestAprilTagID () {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("Widest Tag ID").getInteger(_tagID);
        return _tagID;
    }
}