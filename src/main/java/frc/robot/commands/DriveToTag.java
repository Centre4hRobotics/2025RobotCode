// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTag extends Command {

  private Drive _drive;
  private Vision _vision;

  private double _cameraRotationToTag, _cameraDistanceToTagX, _cameraDistanceToTagY;

  private double _offsetX, _offsetY;
  
  private double _laserDistanceToTagX;

  private PIDController _tagHeadingPIDController; 
  private PIDController _tagDriveXPIDController;
  private PIDController _tagDriveYPIDController;

  private Boolean _isFinished;
  
  /** Creates a new DriveToTag. */
  public DriveToTag(Drive drive, Vision vision, double offsetX, double offsetY) {
    _drive = drive;
    _vision = vision;
    _offsetX = offsetX;
    _offsetY = offsetY;
    _laserDistanceToTagX = -42.0;

    _tagHeadingPIDController = new PIDController(VisionConstants.tagTurningP, VisionConstants.tagTurningI, VisionConstants.tagTurningD);
    _tagDriveXPIDController = new PIDController(VisionConstants.tagDriveXP, VisionConstants.tagDriveXI, VisionConstants.tagDriveXD);
    _tagDriveYPIDController = new PIDController(VisionConstants.tagDriveYP, VisionConstants.tagDriveYI, VisionConstants.tagDriveYD);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform2d position = _vision.getCameraToAprilTag();
    if(position != null) {
      _cameraRotationToTag = position.getRotation().getRadians();
      _cameraDistanceToTagX = position.getX();
      _cameraDistanceToTagY = position.getY();
      _laserDistanceToTagX = _vision.getLaserDistance();
      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      nt.getTable("AprilTag Vision").getEntry("laser reading").setValue(_laserDistanceToTagX);

      
       double velocityX, velocityY, velocityTheta;

       velocityY = _tagDriveYPIDController.calculate(_cameraDistanceToTagY + _cameraDistanceToTagX * -Math.sin(Math.PI + _cameraRotationToTag));

       if(_laserDistanceToTagX > 0) {
        velocityX = _tagDriveXPIDController.calculate(_laserDistanceToTagX + .5);
       } else  {
        velocityX = _tagDriveXPIDController.calculate(_cameraDistanceToTagX + .5);
       }
        velocityTheta = _tagHeadingPIDController.calculate(_cameraRotationToTag);

        if (Math.abs(_cameraRotationToTag) > 3.05)
        {
          velocityTheta = 0.0;
        }

       _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(-velocityX, -velocityY, velocityTheta)); 
       _isFinished = false;
    } else if(_laserDistanceToTagX > 0) {
      double velocityX = _tagDriveXPIDController.calculate(_laserDistanceToTagX + .5);
      _drive.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(-velocityX, 0, 0)); 
    } else
    
    {
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
}
