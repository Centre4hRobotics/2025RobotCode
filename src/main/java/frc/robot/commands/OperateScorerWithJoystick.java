package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scorer;

public class OperateScorerWithJoystick extends Command {

  private Scorer _scorer;
  private Joystick _joystick;
  private BooleanSupplier _override;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public OperateScorerWithJoystick(Scorer scorer, Joystick joystick, BooleanSupplier override) {
    _scorer = scorer;
    _joystick = joystick;
    _override = override;

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

      double rotation = _scorer.getSetpoint();
    double input = -_joystick.getY();

    if (input > 0.2) {
      _scorer.setRotation(rotation + 0.1);
    } else if (input < -0.2 && (rotation >= 0.0 || _override.getAsBoolean())) {
      _scorer.setRotation(rotation - 0.1);
    } else {
      _scorer.setRotation(rotation);
    }
    // if(_controller.getLeftTriggerAxis() > 0)
    //     _scorer.setRotation(rotation + 0.4 * _controller.getLeftTriggerAxis());
    // else
    //     _scorer.setRotation(rotation + -0.4 * _controller.getRightTriggerAxis());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
