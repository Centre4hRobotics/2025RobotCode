package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Scorer;

//ONLY CALL IN CORAL MODE !!
public class SetupToScoreReef extends Command {
    private Elevator _elevator;
    private Scorer _scorer;
    private int _reefLevel;

    private boolean _isFinished;
    
    public SetupToScoreReef(Elevator elevator, Scorer scorer, int reefLevel) {
        _elevator = elevator;
        _scorer = scorer;
        _reefLevel = reefLevel;

        _isFinished = false;

        addRequirements(elevator, scorer);
    }
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double elevatorHeight = ElevatorConstants.heightCoralReef[_reefLevel];
      double scorerRotation = ScorerConstants.rotationEncoderValuesReef[_reefLevel];
      _elevator.setHeight(elevatorHeight);
      _scorer.setRotation(scorerRotation);

      _isFinished = _elevator.isOnTarget(elevatorHeight) && _scorer.isOnTarget(scorerRotation);
    }

    @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
