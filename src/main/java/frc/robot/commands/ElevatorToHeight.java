package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


public class ElevatorToHeight extends Command {
    private Elevator _elevator;
    private double _height;

    private boolean _isFinished;
    
    // level starts counting at 1
    public ElevatorToHeight(Elevator elevator, double height) {
        _elevator = elevator;
        _height = height;

        _isFinished = false;

        addRequirements(elevator);
    }
    @Override
    public void initialize() {
      _elevator.setHeight(_height);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
