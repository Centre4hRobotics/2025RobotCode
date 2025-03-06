// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Drive;

public class DriveWithSpeed extends Command {

  private Drive _driveSubsystem;
  private double _speed; // meters/second

  
  /**
   * This command is responsible for driving in the X direction at a certain speed.
   * Note: Used to test drive feed-forward.
   * 
   * @param driveSubsystem
   * @param speed
   * 
   * Comment By: EternalSyntaxError
   */
  public DriveWithSpeed(Drive driveSubsystem, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveSubsystem = driveSubsystem;
    _speed = speed / RobotConstants.maxSlowDriveSpeed; // because it's being sped up in drive....

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _driveSubsystem.setDesiredRobotRelativeSpeeds(new ChassisSpeeds(_speed, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveSubsystem.freezeWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}