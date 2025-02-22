// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTag extends Command {

  private Drive _drive;
  private Vision _vision;
  private String _side;

  private double _cameraRotationToTag, _cameraDistanceToTagX, _cameraDistanceToTagY;

  private double _offsetX, _offsetY;
  
  private double _laserDistanceToTagX;

  private PIDController _tagHeadingPIDController; 
  private PIDController _tagDriveXPIDController;
  private PIDController _tagDriveYPIDController;
  private PIDController _tagDriveLaserPIDController;

  private Boolean _isFinished;
  
  /** Creates a new DriveToTag. */
  public DriveToTag(Drive drive, Vision vision, double offsetX, double offsetY, String side) {
    _drive = drive;
    _vision = vision;
    _offsetX = offsetX;
    _offsetY = offsetY;
    _side = side;
    _laserDistanceToTagX = -42.0;

    _tagHeadingPIDController = new PIDController(VisionConstants.tagTurningP, VisionConstants.tagTurningI, VisionConstants.tagTurningD);
    _tagDriveXPIDController = new PIDController(VisionConstants.tagDriveXP, VisionConstants.tagDriveXI, VisionConstants.tagDriveXD);
    _tagDriveYPIDController = new PIDController(VisionConstants.tagDriveYP, VisionConstants.tagDriveYI, VisionConstants.tagDriveYD);
    _tagDriveLaserPIDController = new PIDController(VisionConstants.laserTurningP, VisionConstants.laserTurningI, VisionConstants.laserTurningD);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;

    _vision.setCurrentSide(_side);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d position = _vision.getCameraToAprilTag();
    _laserDistanceToTagX = _vision.getLaserDistance();
    
    double velocityX, velocityY, velocityTheta;

    if (_laserDistanceToTagX > 0 && _laserDistanceToTagX < -1.0)
    {
      _cameraRotationToTag = _vision.getLaserDifference();
      if (Math.abs(_cameraRotationToTag) < 5.0 && position != null)
      {
        _cameraRotationToTag = position.getRotation().getRadians();
      } else if (Math.abs(_cameraRotationToTag) < 0.1)
      {
        _cameraRotationToTag = 0.0;
      }

      _cameraDistanceToTagX = _vision.getLaserDistance();

      velocityX = _tagDriveXPIDController.calculate(_cameraDistanceToTagX + 0.1);
      velocityTheta = _tagDriveLaserPIDController.calculate(_cameraRotationToTag);

      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(-velocityX, 0, -velocityTheta));
    }
    else if(position != null) {
      _cameraRotationToTag = position.getRotation().getRadians();
      _cameraDistanceToTagX = position.getX();
      _cameraDistanceToTagY = position.getY();

      velocityY = _tagDriveYPIDController.calculate(_cameraDistanceToTagY);
      velocityX = _tagDriveXPIDController.calculate(_cameraDistanceToTagX - 0.9);
      velocityTheta = _tagHeadingPIDController.calculate(_cameraRotationToTag);

      if (Math.abs(_cameraRotationToTag) < 0.05)
        velocityTheta = 0.0; 

      if (Math.abs(_cameraDistanceToTagY) < 0.15)
        velocityY = 0.0;

      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(0 * -velocityX, -velocityY, velocityTheta)); 
    } else {
      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    // if(_laserDistanceToTagX < VisionConstants.laserDistanceToReef && _laserDistanceToTagX > 0)
    // {
    //   _isFinished = true;
    //   _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(0, 0, 0));
    // } 
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
}