package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class OperateElevatorWithJoystick extends Command {

  private Elevator _elevator;
  private Joystick _joystick;
  private BooleanSupplier _override;
  private BooleanSupplier _mode;

  /**
   * This command is responsible for teleop drive.
   * 
   * @param driveSubsystem
   * @param controller
   * 
   * Comment By: EternalSyntaxError
   */
  public OperateElevatorWithJoystick(Elevator elevator, Joystick joystick, BooleanSupplier override, BooleanSupplier mode) {
    _elevator = elevator;
    _joystick = joystick;
    _override = override;
    _mode = mode;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    // if(!_mode.getAsBoolean()) {
    // double input = -_joystick.getX();
    // double height = _elevator.getSetpoint();
    //   if(input > 0.5 && (height < ElevatorConstants.maxHeight - 0.5 || _override.getAsBoolean())) {
    //     height += 0.5;
    //     // if(height > ElevatorConstants.maxHeight) {
    //     //   height = ElevatorConstants.maxHeight;
    //     // }
    //     _elevator.setHeight(height);
    //   } else if(input < -0.5 && (height > 0.5 || _override.getAsBoolean())) {
    //     height -= 0.5;
    //     // if(height < 0) {
    //     //   height = 0;
    //     // }
    //     _elevator.setHeight(height);
    //   }
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
