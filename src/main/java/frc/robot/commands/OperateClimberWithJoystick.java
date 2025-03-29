package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class OperateClimberWithJoystick extends Command {

  private Climb _climb;
  private Joystick _joystick;
  private BooleanSupplier _climbMode;
  private BooleanSupplier _overrideStops;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public OperateClimberWithJoystick(Climb climb, Joystick joystick, BooleanSupplier climbMode,BooleanSupplier overrideStops) {
    _climb = climb;
    _joystick = joystick;
    _climbMode = climbMode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    if(_climbMode.getAsBoolean()) {
      double rotation = _climb.getSetpoint();
    double input = -_joystick.getX();

    if (input > 0.2) {
      _climb.setPosition(rotation + 3);
    } else if (input < -0.2) {
      if(rotation - 3 < ClimbConstants.manualSoftstop && !_overrideStops.getAsBoolean()) {
        _climb.setPosition(rotation);
      } else {
        _climb.setPosition(rotation - 3);
      }
    } else {
      _climb.setPosition(rotation);
    }
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
