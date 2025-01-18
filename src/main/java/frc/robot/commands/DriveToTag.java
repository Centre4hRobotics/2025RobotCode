// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTag extends Command {

  private Drive _driveSubsystem;
  private Vision _vision;

  private Transform2d _position;
  private double _rotation, _posX, _posY;

  private Transform3d _tagFieldTransform3d;
  private double _deltaX, _deltaY;

  private PIDController _tagHeadingPIDController; 
  private PIDController _tagDriveXPIDController;
  private PIDController _tagDriveYPIDController;
  
  /** Creates a new DriveToTag. */
  public DriveToTag(Drive driveSubsystem, Vision vision, Transform3d tagFieldTransform3d, double deltaX, double deltaY) {
    _driveSubsystem = driveSubsystem;
    _vision = vision;
    _position = _vision.getCameraToAprilTag();
    _rotation = _position.getRotation().getDegrees();
    _posX = _position.getX();
    _posY = _position.getY();

    _deltaX = deltaX;
    _deltaY = deltaY;

    _tagHeadingPIDController = new PIDController(VisionConstants.tagTurningP, VisionConstants.tagTurningI, VisionConstants.tagTurningD);
    _tagDriveXPIDController = new PIDController(VisionConstants.tagDriveXP, VisionConstants.tagDriveXI, VisionConstants.tagDriveXD);
    _tagDriveYPIDController = new PIDController(VisionConstants.tagDriveYP, VisionConstants.tagDriveYI, VisionConstants.tagDriveYD);
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Update transform from vision

    double velocityX = _tagDriveXPIDController.calculate(_posX-_deltaX);
    double velocityY = _tagDriveYPIDController.calculate(_posY-_deltaY);
    double velocityTheta = _tagHeadingPIDController.calculate(_rotation);
    
    _driveSubsystem.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(velocityX, velocityY, velocityTheta));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
