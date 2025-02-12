package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;


public class ElevatorToHeight extends Command {
    private Elevator _elevator;
    private int _level;
    private BooleanSupplier _mode;

    private double _height;

    private boolean _isFinished;
    
    // level starts counting at 1
    public ElevatorToHeight(Elevator elevator, int level, BooleanSupplier mode) {
        _elevator = elevator;
        _level = level;
        _mode = mode;

        _isFinished = false;

        addRequirements(elevator);
    }
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      boolean coral = _mode.getAsBoolean();
      if(coral) { _height = ElevatorConstants.heightCoralReef[_level - 1]; } 
      else { _height = ElevatorConstants.heightAlgae[_level - 1]; }
      
      _elevator.setHeight(_height);
      _isFinished = _elevator.isOnTarget(_height);
    }

    @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return _isFinished;
  }
}
