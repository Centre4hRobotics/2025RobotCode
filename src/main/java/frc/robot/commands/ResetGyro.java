// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;


public class ResetGyro extends Command {

  private Drive _drive;
  /**
   * Resets current gyro angle to 0.
   * 
   * @param drive
   * 
   * Comment By: EternalSyntaxError
   */
  public ResetGyro(Drive drive) {
    _drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(_drive.getSide()) {
      _drive.setHeading(180);
    } else {
      _drive.setHeading(0);
    }
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