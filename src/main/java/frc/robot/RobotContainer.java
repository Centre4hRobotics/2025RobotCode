// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.Drive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive _driveSubsystem = new Drive();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final Joystick _functionJoystick = new Joystick(1);
  private final Joystick _functionJoystick2 = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Button board configuration 
    configureBindings();

    // smartdashboard dropdown
    autoChooserInit();
    
    // Smartdashboard dropdown 
    // error because doesn't exist lol
    
    // Resetting encoders
    _driveSubsystem.syncEncoders();
    
    // Logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void startTeleop() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      _driveSubsystem.flipGyro();
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Configuring teleoperated control
    _driveSubsystem.setDefaultCommand(
      new DriveWithJoystick(_driveSubsystem, m_driverController)    
    );

    // Size is up to max id, not number of buttons
    JoystickButton[] buttonBoard1 = new JoystickButton[9];
    JoystickButton[] buttonBoard2 = new JoystickButton[13];

    // Configuring remaining buttonboard buttons
    for (int i = 1; i <= 8; i++) {
      buttonBoard1[i] = new JoystickButton(_functionJoystick, i);
    } 
    for (int i = 1; i <= 7; i++) {
      buttonBoard2[i] = new JoystickButton(_functionJoystick2, i);
    }
    for (int i = 9; i <= 12; i++) {
      buttonBoard2[i] = new JoystickButton(_functionJoystick2, i);
    }

    // Syncs encoders   
    m_driverController.b().onTrue(
      Commands.runOnce(() -> _driveSubsystem.syncEncoders(), _driveSubsystem) 
    );

    m_driverController.y().onTrue(new ResetGyro(_driveSubsystem));

    // In order to properly run characterization tests, it is best to be able to
    // manually control the stop/stop of the logger to remove as much noise.
    m_driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    m_driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    buttonBoard1[5].whileTrue(_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    buttonBoard1[6].whileTrue(_driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    buttonBoard1[8].whileTrue(_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    buttonBoard1[7].whileTrue(_driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // m_driverController.povUp().onTrue(
    //   Commands.runOnce(() -> _driveSubsystem.setDesiredHeading(0), _driveSubsystem)
    // );
    // m_driverController.povRight().onTrue(
    //   Commands.runOnce(() -> _driveSubsystem.setDesiredHeading(270), _driveSubsystem)
    // );
    // m_driverController.povDown().onTrue(
    //   Commands.runOnce(() -> _driveSubsystem.setDesiredHeading(180), _driveSubsystem)
    // );
    // m_driverController.povLeft().onTrue(
    //   Commands.runOnce(() -> _driveSubsystem.setDesiredHeading(90), _driveSubsystem)
    // );
  }

  public void autoChooserInit() {
    String[] autoselector = {
      "drive 3m",
      "d4,c4,b4"
    };
    SmartDashboard.putStringArray("Auto List", autoselector);
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selection = SmartDashboard.getString("Auto Selector", "None");
    
    Command autoCommand = Commands.runOnce(
      () -> _driveSubsystem.freezeWheels(), _driveSubsystem
    );  //The default command will be to freeze if nothing is selected

    autoCommand = new PathPlannerAuto(selection);
    return autoCommand;
  }
}
