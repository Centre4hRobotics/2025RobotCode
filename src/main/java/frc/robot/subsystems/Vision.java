package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private double _posX; // "Pose X"
    private double _posY; // "Pose Y"
    private double _rotation; // "Tag Rotation"
    private int _tagID; // "Widest Tag ID"
    private boolean _tagPresent; // "AprilTag Presence"

    public Transform2d getCameraToAprilTag() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("AprilTag Presence").getBoolean(_tagPresent);
        if(_tagPresent == true) {
            nt.getTable("AprilTag Vision").getEntry("Pose X").getDouble(_posX);
            nt.getTable("AprilTag Vision").getEntry("Pose Y").getDouble(_posY);
            nt.getTable("AprilTag Vision").getEntry("Tag Rotation").getDouble(_rotation);
            if(_rotation < 0) {_rotation += Math.PI;} else {_rotation -= Math.PI;}
            return new Transform2d(_posX, _posY, new Rotation2d(_rotation));
        }
        return null;
    }

    public int getBestAprilTagID () {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("Widest Tag ID").getInteger(_tagID);
        return _tagID;
    }
}