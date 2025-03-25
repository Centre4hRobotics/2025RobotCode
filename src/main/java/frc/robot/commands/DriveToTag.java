// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTag extends Command {

  private Drive _drive;
  private Vision _vision;
  private String _side;
  private int _tagID;

  private double _cameraRotationToTag, _cameraDistanceToTagX, _cameraDistanceToTagY;
  
  private double _laserDistanceToTagX, _laserRotationToTag;

  private double _offsetY;

  private PIDController _visionHeadingPID; 
  private PIDController _visionDriveXPID;
  private PIDController _visionDriveYPID;

  private Boolean _isFinished;
  
  /** Creates a new DriveToTag. */
  public DriveToTag(Drive drive, Vision vision, String side, String location) {
    _drive = drive;
    _vision = vision;
    _side = side;
    _tagID = getTagID(location);
    _laserDistanceToTagX = -42.0;
    _offsetY = VisionConstants.centeredReefDeltaY;
    if(_side.equals("LEFT")) {_offsetY *= -1; }

    _visionHeadingPID = new PIDController(VisionConstants.tagTurningP, VisionConstants.tagTurningI, VisionConstants.tagTurningD);
    _visionDriveXPID = new PIDController(VisionConstants.tagDriveXP, VisionConstants.tagDriveXI, VisionConstants.tagDriveXD);
    _visionDriveYPID = new PIDController(VisionConstants.tagDriveYP, VisionConstants.tagDriveYI, VisionConstants.tagDriveYD);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;

    _vision.setCurrentSide(_side);
    _vision.setCurrentTagID(_tagID);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d position = _vision.getCameraToAprilTag();
    _laserDistanceToTagX = _vision.getLaserDistance();
    
    double velocityX, velocityY, velocityTheta;

    if (_laserDistanceToTagX > 0)
    {
      if (position != null)
      {
        _cameraRotationToTag = -position.getRotation().getRadians();
        _cameraDistanceToTagY = position.getY();

        velocityTheta = _visionHeadingPID.calculate(_cameraRotationToTag);
        velocityY = -_visionDriveYPID.calculate(_cameraDistanceToTagY + _offsetY);

      } else {
        _laserRotationToTag = _vision.getLaserAngle();
        _laserDistanceToTagX = _vision.getLaserDistance();
        
        velocityTheta = _visionHeadingPID.calculate(_laserRotationToTag);
        velocityY = 0;
      }

      velocityX = _visionDriveXPID.calculate(_laserDistanceToTagX);

      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(-velocityX, velocityY, -velocityTheta));
    }
    else if(position != null) {
      _cameraRotationToTag = position.getRotation().getRadians();
      _cameraDistanceToTagX = position.getX();
      _cameraDistanceToTagY = position.getY();

      velocityY = _visionDriveYPID.calculate(_cameraDistanceToTagY + _offsetY);
      velocityX = _visionDriveXPID.calculate(_cameraDistanceToTagX);
      velocityTheta = _visionHeadingPID.calculate(_cameraRotationToTag);

      if (Math.abs(_cameraRotationToTag) < 0.03)
        velocityTheta = 0.0; 

      if (Math.abs(_cameraDistanceToTagY) < 0.01)
        velocityY = 0.0;

      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(-velocityX, -velocityY, velocityTheta)); 
    } else {
      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    if(Math.abs(_laserDistanceToTagX) < VisionConstants.laserDistanceToReef && _laserDistanceToTagX > 0)
    {
      _isFinished = true;
      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0));
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.setDesiredHeading(_drive.getHeading());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinished;
  }

  public int getTagID(String location) {
    if(location.equals(null)) { return 0; }
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        switch(location) {
          case "ab":
            return 7;
          case "cd":
            return 8;
          case "ef":
            return 9;
          case "gh":
            return 10;
          case "ij":
            return 11;
          case "kl":
            return 6;
        }
      } else {
        switch(location) {
          case "ab":
            return 18;
          case "cd":
            return 17;
          case "ef":
            return 22;
          case "gh":
            return 21;
          case "ij":
            return 20;
          case "kl":
            return 19;
      }
    }
  } return 0;
  }
}