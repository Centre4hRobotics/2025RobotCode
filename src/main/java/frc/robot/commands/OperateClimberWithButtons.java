package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climb;

public class OperateClimberWithButtons extends Command {

  private Climb _climb;
  private boolean _forwards;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public OperateClimberWithButtons(Climb climb, boolean forwards) {
    _climb = climb;
    _forwards = forwards;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotation = _climb.getSetpoint();

    if (_forwards) {
        _climb.setRotation(rotation + 0.1);
    } else {
        _climb.setRotation(rotation - 0.1);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
