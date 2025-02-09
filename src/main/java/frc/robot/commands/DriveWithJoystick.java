// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;

public class DriveWithJoystick extends Command {

  private Drive _driveSubsystem;
  private CommandXboxController _controller;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public DriveWithJoystick(Drive driveSubsystem, CommandXboxController controller) {
    _driveSubsystem = driveSubsystem;
    _controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // _driveSubsystem.syncEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // robot centric (joystick x decides direction of angular velocity)
    double joystickAngularVelocityInput = -_controller.getLeftX();
    if (Math.abs(joystickAngularVelocityInput) < .2) { // deadzone
      joystickAngularVelocityInput = 0;
    }
    else {
      joystickAngularVelocityInput = Math.signum(joystickAngularVelocityInput) * Math.pow(((Math.abs(joystickAngularVelocityInput) - .2) * 1.25), 2);
    }

    // weird because x axis is forwards
    double joystickXSpeedInput = -_controller.getRightY();
    double joystickYSpeedInput = -_controller.getRightX();

    // finds the magnitude
    double totalThrottle = Math.hypot(joystickXSpeedInput, joystickYSpeedInput);
    
    if (totalThrottle < .2) { // deadzone
      joystickXSpeedInput = 0; joystickYSpeedInput = 0;
    } else {
      //scales each component by using joystick angle
      double joystickAngle = Math.atan2(joystickYSpeedInput, joystickXSpeedInput);
      totalThrottle = Math.pow(totalThrottle, 2.3); //(1-_elevator.heightFraction())
      joystickXSpeedInput = totalThrottle * Math.cos(joystickAngle);
      joystickYSpeedInput = totalThrottle * Math.sin(joystickAngle);
    }

    _driveSubsystem.drive(joystickXSpeedInput, joystickYSpeedInput, joystickAngularVelocityInput, _controller.getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveSubsystem.setDesiredStates(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
