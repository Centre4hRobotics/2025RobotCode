package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Scorer;


public class RotateScorer extends Command {
    private Scorer _scorer;
    private double _rotation;
    private int _level;
    private BooleanSupplier _mode;

    private boolean _isFinished;
    
    public RotateScorer(Scorer scorer, int level, BooleanSupplier mode) {
        _scorer = scorer;
        _mode = mode;
        _level = level;

        _isFinished = false;

        addRequirements(scorer);
    }
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      boolean coral = _mode.getAsBoolean();
      if(coral) { _rotation = ScorerConstants.rotationsCoral[_level - 1]; } 
      else { _rotation = ScorerConstants.rotationsAlgae[_level - 1]; }
      
      _scorer.setRotation(_rotation);
      _isFinished = _scorer.isOnTarget(_rotation);
    }

    @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
