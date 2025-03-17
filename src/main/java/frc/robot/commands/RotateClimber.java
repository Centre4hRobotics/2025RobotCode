package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;


public class RotateClimber extends Command {
    
    private boolean _isFinished;
    private Climb _climb;
    private double _position;

    
    public RotateClimber(Climb climb, double position) {
        
        _climb = climb;
        _isFinished = false;
        _position = position;

        addRequirements(climb);
    }
    @Override
    public void initialize() {
      _climb.setPosition(_position);
      _isFinished = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      _isFinished = _climb.isOnTarget(_position);
    }

    @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
