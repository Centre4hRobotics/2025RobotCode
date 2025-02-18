package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;

public class IntakeCoralUntilIn extends Command {

    private Scorer _scorer;

    private boolean _coralIn;
    private boolean _isFinished;
    private double _startingPos;

    public IntakeCoralUntilIn(Scorer scorer) {
        _scorer = scorer;
        _coralIn = false;
        _isFinished = false;
        _startingPos = 0;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean temp = _coralIn;
    _coralIn = _scorer.getScoringCurrent() > ScorerConstants.scoringCurrentWithCoral;
    if(_coralIn && !temp) {
      _startingPos = _scorer.getScoringPosition();
    } else if(_coralIn) {
      if(_scorer.getScoringPosition() > _startingPos + 1) {
        _isFinished = true;
      } else {
        _scorer.setScoringVoltage(8);
      }
    } else {
      _scorer.setScoringVoltage(8);
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
