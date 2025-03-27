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

    private double _posToTagX; 
    private double _posToTagY; 
    private double _rotationToTag; 

    private boolean _tagPresent; 
    private String _side;
    private int _tagID;

    private DoubleSubscriber _posToTagXSub; // “Tag Relative Pose X”
    private DoubleSubscriber _posToTagYSub; // “Tag Relative Pose Y”
    private DoubleSubscriber _rotationToTagSub; // Tag Relative Rotation”

    private BooleanSubscriber _tagPresenceSub; // "AprilTag Presence"

    private LaserCan _rightLaser;
    private LaserCan _leftLaser;

    public Vision() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("AprilTag Vision");

        _tagPresenceSub = table.getBooleanTopic("AprilTag Presence").subscribe(false);

        _rotationToTagSub = table.getDoubleTopic("TagToCamera Theta").subscribe(0);
        _posToTagXSub = table.getDoubleTopic("TagToCamera X").subscribe(0);
        _posToTagYSub = table.getDoubleTopic("TagToCamera Y").subscribe(0);        

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
        table.getEntry("Tag Choice").setValue(_tagID);
        _tagPresent = _tagPresenceSub.get();
        if(_tagPresent == true) {
            _rotationToTag = _rotationToTagSub.get();
            _posToTagX = _posToTagXSub.get();
            _posToTagY = _posToTagYSub.get();
            return new Transform2d(_posToTagX, _posToTagY, new Rotation2d(_rotationToTag));
        }
        return null;
    }

    public void setCurrentSide(String side)
    {
        _side = side;
    }

    public void setCurrentTagID(int tagID)
    {
        _tagID = tagID;
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

    public double getLaserAngle()
    {
        double diff = getRightLaserDistance() - getLeftLaserDistance();
        return Math.atan2(diff, 0.34);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("AprilTag Vision").getEntry("laser angle").setValue(getLaserAngle()*180/Math.PI); 
        nt.getTable("AprilTag Vision").getEntry("laser distance").setValue(getLaserDistance()); 
    }
}