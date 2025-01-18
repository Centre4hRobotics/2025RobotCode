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
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToTag extends Command {

  private Drive _driveSubsystem;
  private Transform3d _tagFieldTransform3d;
  private PIDController _tagHeadingPIDController;
  private double _deltaX, _deltaY;
  
  /** Creates a new DriveToTag. */
  public DriveToTag(Drive driveSubsystem, Transform3d tagFieldTransform3d, double deltaX, double deltaY) {
    _driveSubsystem = driveSubsystem;
    _tagFieldTransform3d = tagFieldTransform3d;
    _tagHeadingPIDController = MotorConstants.tagHeadingPIDController;
    _deltaX = deltaX;
    _deltaY = deltaY;
    
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

    // TODO: Add offset for camera position/bumper position/height/etc
    Transform3d robotFieldTransform3d = _tagFieldTransform3d.inverse().plus(new Transform3d(_deltaX, _deltaY, 0, new Rotation3d(0.0, 0.0, 0.0)));
    

    double velocityX = 0;//_tagXPIDController.calculate(robotFieldTransform3d.getX());
    double velocityY = 0;
    double velocityTheta = _tagHeadingPIDController.calculate(robotFieldTransform3d.getRotation().getAngle());
    
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
