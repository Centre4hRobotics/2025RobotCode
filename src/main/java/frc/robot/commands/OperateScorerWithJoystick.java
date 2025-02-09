package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Scorer;

public class OperateScorerWithJoystick extends Command {

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
  public OperateScorerWithJoystick(Scorer scorer, CommandXboxController controller) {
    _scorer = scorer;
    _controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_scorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(_controller.getRightX()) > 0.2) {
      _scorer.setRotationVoltage(_controller.getRightX() * 9);
    } else {
      _scorer.setRotationVoltage(0);
    }

    if(_controller.getLeftTriggerAxis() > 0)
        _scorer.setScoringVoltage(_controller.getLeftTriggerAxis() * 12);
    else
        _scorer.setScoringVoltage(_controller.getRightTriggerAxis() * -12);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
