package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;

public class IntakeCoralUntilIn extends Command {

    private Scorer _scorer;

    private boolean _coralIn;
    private static enum CoralStatus
    {
      spiningUp,
      waitingForCoral,
      hasCoral,
      coralReady
    }

    private CoralStatus _state;
    private boolean _isFinished;
    private double _startingPos;

    public IntakeCoralUntilIn(Scorer scorer) {
        _scorer = scorer;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _state = CoralStatus.spiningUp;
    _isFinished = false;
    _startingPos = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (_state)
    {
      case spiningUp:
      _scorer.setScoringVoltage(ScorerConstants.intakingCoralVoltage);
        if(_scorer.getScoringVelocity() > 1200)
          _state = CoralStatus.waitingForCoral;
        break;
      case waitingForCoral:
        if (_scorer.getScoringVelocity() < 800)
        {
          _state = CoralStatus.hasCoral;
          _startingPos = _scorer.getScoringPosition();
        }
        break;
      case hasCoral:
        if(_scorer.getScoringPosition() < _startingPos + ScorerConstants.numRotationsToIntake)
          _scorer.setScoringVoltage(ScorerConstants.intakingCoralVoltage);
        else
        {
          _state = CoralStatus.coralReady;
          _scorer.setScoringVoltage(0.0);
        }
        break;
      case coralReady:
        _isFinished = true;
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
