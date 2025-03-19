package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Scorer;


public class RotateScorer extends Command {
    private Scorer _scorer;
    private double _rotation;

    private boolean _isFinished;
    
    public RotateScorer(Scorer scorer, double rotation) {
        _scorer = scorer;
        _rotation = rotation;

        _isFinished = false;

        addRequirements(scorer);
    }
    @Override
    public void initialize() {
      _scorer.setRotation(_rotation);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
