package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

public class OperateElevatorWithJoystick extends Command {

  private Elevator _elevator;
  private CommandXboxController _controller;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public OperateElevatorWithJoystick(Elevator elevator, CommandXboxController controller) {
    _elevator = elevator;
    _controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _elevator.setVoltage(_controller.getLeftY() * 0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
