package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
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
    if (_forwards) {
        _climb.setVoltage(10.0);
    } else {
        _climb.setVoltage(-10.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    _climb.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}