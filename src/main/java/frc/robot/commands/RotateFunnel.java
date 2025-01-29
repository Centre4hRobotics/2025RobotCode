package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FunnelConstants;
import frc.robot.subsystems.Funnel;

public class RotateFunnel extends Command {
    private Funnel _funnel;
    private BooleanSupplier _funnelRotation;

    public RotateFunnel(Funnel funnel, BooleanSupplier funnelRotation) {
        _funnel = funnel;
        _funnelRotation = funnelRotation;
        addRequirements(funnel);
    }

     @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(_funnelRotation.getAsBoolean()) {
            _funnel.setHeight(FunnelConstants.heightStation);
        } else {
            _funnel.setHeight(FunnelConstants.heightClimb);
        }
    }

    @Override
  public void end(boolean interrupted) {}
}