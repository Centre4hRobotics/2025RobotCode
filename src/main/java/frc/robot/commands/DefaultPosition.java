package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Scorer;

public class DefaultPosition extends Command {
    private Elevator _elevator;
    private Funnel _funnel;
    private Scorer _scorer;

    public DefaultPosition(Elevator elevator, Funnel funnel, Scorer scorer) {
        _elevator = elevator;
        _funnel = funnel;
        _scorer = scorer;
    }
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        _elevator.setHeight(ElevatorConstants.heightDefault);
        _funnel.setHeight(FunnelConstants.heightStation);
        _scorer.setRotation(ScorerConstants.rotationDefault);
    }

    @Override
  public void end(boolean interrupted) {}
}
