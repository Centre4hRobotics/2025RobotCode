package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;

public class IntakeCoralUntilIn extends Command {

    private Scorer _scorer;
    private boolean _isFinished;

    private static enum CoralStatus
    {
      spiningUp,
      backdriving
    }

    private CoralStatus _state;
    private double _startingPos;

    public IntakeCoralUntilIn(Scorer scorer) {
        _scorer = scorer;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;
    _state = CoralStatus.spiningUp;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(_state) {
      case spiningUp:
        _scorer.setScoringVoltage(ScorerConstants.autoIntakingCoralVoltage);
        if(_scorer.getCoralIn()) {
          _startingPos = _scorer.getScoringPosition();
          _state = CoralStatus.backdriving;
        }
        break;
      case backdriving:
        _scorer.setScoringVoltage(ScorerConstants.autoBackdrivingCoralVoltage);
        _isFinished = _startingPos - _scorer.getScoringPosition() > ScorerConstants.numRotationsAutoBackdrive;
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _scorer.setScoringVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
