package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;

public class ManipulateGamePiece extends Command {

    Scorer _scorer;
    BooleanSupplier _scoringModeSwitch;
    boolean _forwards, _stuck = false;;

    public ManipulateGamePiece(Scorer scorer, BooleanSupplier scoringModeSwitch, boolean forwards) {
        _scorer = scorer;
        _scoringModeSwitch = scoringModeSwitch;
        _forwards = forwards;
    }

    public ManipulateGamePiece(Scorer scorer, BooleanSupplier scoringModeSwitch, boolean forwards, boolean stuck) {
      _scorer = scorer;
      _scoringModeSwitch = scoringModeSwitch;
      _forwards = forwards;
      _stuck = stuck;
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
        if (_stuck) {
          _scorer.setScoringVoltage(ScorerConstants.stuckCoralVoltage);
        }
        else {
        _scorer.setScoringVoltage(ScorerConstants.intakingCoralVoltage);
        }
      } else {
        _scorer.setScoringVoltage(ScorerConstants.backdrivingCoralVoltage);
      }
    } else {
      if(_forwards) {
        _scorer.setScoringVoltage(ScorerConstants.ejectingAlgaeVoltage);
      } else {
        _scorer.setScoringVoltage(ScorerConstants.intakingAlgaeVoltage);
      }
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
    return false;
  }
}
