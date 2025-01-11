// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;


public class ResetGyro extends Command {

  private Drive _driveSubsystem;
  /**
   * Resets current gyro angle to 0.
   * 
   * @param drive
   * 
   * Comment By: EternalSyntaxError
   */
  public ResetGyro(Drive drive) {
    _driveSubsystem = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    _driveSubsystem.resetGyroAngle();
    _driveSubsystem.setDesiredHeading(0);
    // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
    //   _driveSubsystem.flipGyro();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}