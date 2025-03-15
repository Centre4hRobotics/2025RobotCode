import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Climb;


public class ClimbAndHold extends Command {
    
    private boolean _isFinished;
    private Climb _climb;

    
    public ClimbAndHold(Climb climb) {
        
        _climb = climb;
        _isFinished = false;

        addRequirements(climb);
    }
    @Override
    public void initialize() {
      _climb.setPosition(ClimbConstants.climbed);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
    }

    @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
