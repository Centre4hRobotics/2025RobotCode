package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;

public class ManipulateGamePiece extends Command {

    Scorer _scorer;
    BooleanSupplier _scoringModeSwitch;
    boolean _forwards;

    public ManipulateGamePiece(Scorer scorer, BooleanSupplier scoringModeSwitch, boolean forwards) {
        _scorer = scorer;
        _scoringModeSwitch = scoringModeSwitch;
        _forwards = forwards;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(_scoringModeSwitch.getAsBoolean()) {
      if(_forwards) {
        _scorer.setScoringVelocity(ScorerConstants.intakingCoralVelocity);
      } else {
        _scorer.setScoringVelocity(ScorerConstants.scoringCoralVelocity);
      }
    } else {
      if(_forwards) {
        _scorer.setScoringVelocity(ScorerConstants.intakingAlgaeVelocity);
      } else {
        _scorer.setScoringVelocity(ScorerConstants.scoringAlgaeVelocity);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
