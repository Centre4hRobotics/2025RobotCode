package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private double _posToTagX; 
    private double _posToTagY; 
    private double _rotationToTag; 
    
    private double _posFieldX; 
    private double _posFieldY; 
    private double _rotationField; 

    private boolean _tagPresent; 
    private String _side;
    private int _tagID;

    private DoubleSubscriber _posToTagXSub; // “Tag Relative Pose X”
    private DoubleSubscriber _posToTagYSub; // “Tag Relative Pose Y”
    private DoubleSubscriber _rotationToTagSub; // Tag Relative Rotation”

    private DoubleSubscriber _posFieldXSub; // “Field Relative Pose X”
    private DoubleSubscriber _posFieldYSub; // “Field Relative Pose Y”
    private DoubleSubscriber _rotationFieldSub; // “Field Relative Rotation X”

    private BooleanSubscriber _tagPresenceSub; // "AprilTag Presence"
    private IntegerSubscriber _tagIDSub;
    
    private LaserCan _rightLaser;
    private LaserCan _leftLaser;

    public Vision(String side) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("AprilTag Vision");
        table.getEntry("Using Camera").setValue(side);
        _side = side;

        _tagPresenceSub = table.getBooleanTopic("AprilTag Presence").subscribe(false);
        _tagIDSub = table.getIntegerTopic("Widest Tag ID").subscribe(0);

        _rotationToTagSub = table.getDoubleTopic("TagToCamera Theta").subscribe(0);
        _posToTagXSub = table.getDoubleTopic("TagToCamera X").subscribe(0);
        _posToTagYSub = table.getDoubleTopic("TagToCamera Y").subscribe(0);

        _rotationFieldSub = table.getDoubleTopic("TagToCamera Theta").subscribe(0);
        _posFieldXSub = table.getDoubleTopic("TagToCamera X").subscribe(0);
        _posFieldYSub = table.getDoubleTopic("TagToCamera Y").subscribe(0);
        

        _rightLaser = new LaserCan(12);
        _leftLaser = new LaserCan(13);
        try {
            _rightLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            _rightLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            _rightLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);

            _leftLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            _leftLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            _leftLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public Transform2d getCameraToAprilTag() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("AprilTag Vision");
        table.getEntry("Using Camera").setValue(_side);
        _tagPresent = _tagPresenceSub.get();
        if(_tagPresent == true) {
            _rotationToTag = _rotationToTagSub.get();
            _posToTagX = _posToTagXSub.get();
            _posToTagY = _posToTagYSub.get();
            return new Transform2d(_posToTagX, _posToTagY, new Rotation2d(_rotationToTag));
        }
        return null;
    }

    public Transform2d getPositionFromTag() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("AprilTag Vision");
        table.getEntry("Using Camera").setValue(_side);
        _tagPresent = _tagPresenceSub.get();
        if(_tagPresent == true) {
            _rotationField = _rotationFieldSub.get();
            _posFieldX = _posFieldXSub.get();
            _posFieldY = _posFieldYSub.get();
            return new Transform2d(_posFieldX, _posFieldY, new Rotation2d(_rotationField));
        }
        return null;
    }

    public String getCurrentSide()
    {
        return _side;
    }

    public void setCurrentSide(String side)
    {
        _side = side;
    }

    public int getBestAprilTagID () {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("Widest Tag ID").getInteger(_tagID);
        return _tagID;
    }

    public double getLeftLaserDistance()
    {
        LaserCan.Measurement measurement = _leftLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){

            return measurement.distance_mm / 1000.0;
            
        } else {
            // System.out.println("RIP left laser, bro tried and failed...");
            return -4027;
        }
    }

    public double getRightLaserDistance()
    {
        LaserCan.Measurement measurement = _rightLaser.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){

            return measurement.distance_mm / 1000.0;
            
        } else {
            // System.out.println("Right laser dun goofed D:");
            return -7204;
        }
    }

    public double getLaserDistance()
    {
        return (getRightLaserDistance() + getLeftLaserDistance()) / 2.0;
    }

    public double getLaserDifference()
    {
        double diff = getRightLaserDistance() - getLeftLaserDistance();
        // if (diff >= 0.0) {
        //     diff = Math.PI - diff;
        // } else {
        //     diff = -Math.PI - diff;
        // }
        return diff;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("laser diff").setValue(getLaserDifference()); 
    }
}