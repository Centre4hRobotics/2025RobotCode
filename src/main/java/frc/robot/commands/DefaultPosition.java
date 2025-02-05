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

    public DefaultPosition(Scorer scorer, Elevator elevator, BooleanSupplier scoringModeSwitch) {
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
        // coral mode
        if (_scoringModeSwitch.getAsBoolean()) {
            // if its not safe to elevate (algae side):
                // CASE 1: its safe to elevate (coral side) and can move to centered coral position w/o leaving safe zone
                // CASE 2: in between coral/algae and NOT safe to elevate, needs to move before it can elevate
            // if its below crossbeam:
                // safe to rotate as needed, rotate to default position :)
            // if its safe to elevate (algae side):
                // stays on algae side (doesnt rotate) until its below crossbeam for efficiency
            if(!_scorer.safeToElevateAlgae() || _elevator.belowCrossbeam()) {
                _scorer.setRotation(ScorerConstants.rotationCoralDefault);
            }
            // sets height when its safe :P
            // coral default is below crossbeam, if its already below crossbeam its safe to go b/w heights without checking scorer
            if(_scorer.safeToElevate() || _elevator.belowCrossbeam()) {
                _elevator.setHeight(ElevatorConstants.heightCoralL1);
            // could just be a regular else but im anxious
            // handles case where its above the crossbeam and scorer is still being rotated out of the way
            // moves as far down as it can in the meantime
            } else if (_elevator.aboveCrossbeam()) {
                _elevator.setHeight(ElevatorConstants.heightAboveCrossbeam);
            }
        // algae mode
        } else {
            if(!_scorer.safeToElevateCoral() || _elevator.belowCrossbeam()) {
                _scorer.setRotation(ScorerConstants.rotationAlgaeDefault);
            }
            if(_scorer.safeToElevate() || _elevator.belowCrossbeam()) {
                _elevator.setHeight(ElevatorConstants.heightAlgaeProcessor);
            // could just be a regular else but im anxious
            } else if (_elevator.aboveCrossbeam()) {
                _elevator.setHeight(ElevatorConstants.heightAboveCrossbeam);
            }
        }
    }

    @Override
  public void end(boolean interrupted) {}
}
