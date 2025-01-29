package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Scorer;

public class DefaultPosition extends Command {
    private Elevator _elevator;
    private Scorer _scorer;
    private BooleanSupplier _scoringModeSwitch;

    public DefaultPosition(Elevator elevator, Scorer scorer, BooleanSupplier scoringModeSwitch) {
        _elevator = elevator;
        _scorer = scorer;
        _scoringModeSwitch = scoringModeSwitch;
        addRequirements(elevator, scorer);
    }
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // coral
        if(_scoringModeSwitch.getAsBoolean()) {
            _elevator.setHeight(ElevatorConstants.heightL1);
            _scorer.setRotation(ScorerConstants.rotationL123);
        } 
        // algae
        else {
            _elevator.setHeight(ElevatorConstants.heightL1);
            _scorer.setRotation(ScorerConstants.rotationAlgae);
        }
    }

    @Override
  public void end(boolean interrupted) {}
}
