package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private double _posX; // "Pose X"
    private double _posY; // "Pose Y"
    private double _rotation; // "Tag Rotation"
    private int _tagID; // "Widest Tag ID"
    private boolean _tagPresent; // "AprilTag Presence"
    private String _side;

    private BooleanSubscriber _tagPresenceSub;
    private DoubleSubscriber _rotationSub;
    private DoubleSubscriber _posXSub;
    private DoubleSubscriber _posYSub;
    
    private LaserCan _laser;

    public Vision(String side) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("AprilTag Vision");
        table.getEntry("Using Camera").setValue(side);
        _tagPresenceSub = table.getBooleanTopic("AprilTag Presence").subscribe(false);
        _rotationSub = table.getDoubleTopic("Tag Rotation").subscribe(0);
        _posXSub = table.getDoubleTopic("Pose X").subscribe(0);
        _posYSub = table.getDoubleTopic("Pose Y").subscribe(0);
        _side = side;
        

        _laser = new LaserCan(12);
        try {
            _laser.setRangingMode(LaserCan.RangingMode.SHORT);
            _laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            _laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public Transform2d getCameraToAprilTag() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("AprilTag Vision");
        table.getEntry("Using Camera").setValue(_side);
        _tagPresent = _tagPresenceSub.get();
        System.out.println(_tagPresent);
        if(_tagPresent == true) {
            _rotation = _rotationSub.get();
            _posX = _posXSub.get();
            _posY = _posYSub.get();
            System.out.println("rotation: " + _rotation + "pos x:" + _posX + "pos y:" + _posY);
            return new Transform2d(_posX, _posY, new Rotation2d(_rotation));
        }
        return null;
    }

    public int getBestAprilTagID () {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("Widest Tag ID").getInteger(_tagID);
        return _tagID;
    }

    public double getLaserDistance()
    {
        LaserCan.Measurement measurement = _laser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return measurement.distance_mm / 1000.0;
        } else {
            System.out.println("Oh no! The target is out of range or we can't get a reliable measurement!");
            return -42;
        }
    }
}