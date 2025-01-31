package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Scorer;

public class OperateWithJoystick extends Command {

  private Elevator _elevator;
  private Scorer _scorer;
  private CommandXboxController _controller;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public OperateWithJoystick(Elevator elevator, Scorer scorer, CommandXboxController controller) {
    _elevator = elevator;
    _scorer = scorer;
    _controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_elevator, _scorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Command moveElevator = new RunCommand(() -> _elevator.setVoltage(_controller.getLeftY() * 0.2), _elevator);
    _controller.leftStick().whileTrue(moveElevator);

    Command moveScorer = new RunCommand(() -> _scorer.setRotationVoltage(_controller.getRightX() * 0.1), _scorer);
    _controller.rightStick().whileTrue(moveScorer);

    Command spinScorerIn = new RunCommand(() -> _scorer.setScoringVoltage(_controller.getLeftTriggerAxis() * 0.5), _scorer);
    _controller.leftTrigger().whileTrue(spinScorerIn);

    Command spinScorerOut = new RunCommand(() -> _scorer.setScoringVoltage(_controller.getRightTriggerAxis() * -0.5), _scorer);
    _controller.rightTrigger().whileTrue(spinScorerOut);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
