package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Scorer;

//ONLY CALL IN CORAL MODE !!
public class ScoreReef extends Command {
    private Elevator _elevator;
    private Scorer _scorer;
    private int _reefLevel;
    
    public ScoreReef(Elevator elevator, Scorer scorer, int reefLevel) {
        _elevator = elevator;
        _scorer = scorer;
        _reefLevel = reefLevel;
        addRequirements(elevator, scorer);
    }
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _elevator.setHeight(ElevatorConstants.heightEncoderValuesReef[_reefLevel]);
        _scorer.setRotation(ScorerConstants.rotationEncoderValuesReef[_reefLevel]);
    }

    @Override
  public void end(boolean interrupted) {}
}
