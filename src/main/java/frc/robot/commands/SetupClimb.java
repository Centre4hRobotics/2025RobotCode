import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Climb;


public class SetupClimb extends Command {
    
    private boolean _isFinished;
    private Climb _climb;

    private static enum ClimbStatus
    {
      droppingFunnel,
      gettingInFinalPosition
    }

    private ClimbStatus _state;
    
    public SetupClimb(Climb climb) {
        
        _climb = climb;
        _isFinished = false;

        addRequirements(climb);
    }
    @Override
    public void initialize() {
      _state = ClimbStatus.droppingFunnel;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      switch(_state) {
      case droppingFunnel:
        _climb.setPosition(ClimbConstants.dropFunnel);
        if(_climb.isOnTarget(ClimbConstants.dropFunnel)) {
          _state = ClimbStatus.gettingInFinalPosition;
        }
        break;
      case gettingInFinalPosition:
        _climb.setPosition(ClimbConstants.prepClimb);
        _isFinished = _climb.isOnTarget(ClimbConstants.prepClimb);
        break;
    }
    }

    @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
