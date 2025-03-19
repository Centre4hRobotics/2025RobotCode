package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;

public class EjectCoralUntilOut extends Command {

    private Scorer _scorer;

    private boolean _isFinished;
    private double _startingPos;

    public EjectCoralUntilOut(Scorer scorer) {
        _scorer = scorer;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;
    _startingPos = _scorer.getScoringPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _scorer.setScoringVoltage(4);
    _isFinished = _scorer.getScoringPosition() > _startingPos + ScorerConstants.numRotationsToEject;
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
