// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultPosition;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ManipulateGamePiece;
import frc.robot.commands.SetupToScoreReef;
import frc.robot.commands.DriveWithSpeed;
import frc.robot.commands.OperateElevatorWithJoystick;
import frc.robot.commands.OperateScorerWithJoystick;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Funnel;
import frc.robot.subsystems.Scorer;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive _drive = new Drive();
  private final Vision _rightCamera = new Vision("RIGHT");
  private final Vision _leftCamera = new Vision("LEFT");
  private final Funnel _funnel = new Funnel();
  private final Elevator _elevator = new Elevator();
  private final Scorer _scorer = new Scorer();
  private final Climb _climb = new Climb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_functionController = 
      new CommandXboxController(OperatorConstants.kFunctionControllerPort);

  //private final Joystick _functionJoystick = new Joystick(1);
  //private final Joystick _functionJoystick2 = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // register pathplanner commands

    BooleanSupplier coral = () -> true;
    BooleanSupplier alage = () -> false;
    NamedCommands.registerCommand("score coral L1", new SetupToScoreReef(_elevator, _scorer, 1)
      .andThen(new ManipulateGamePiece(_scorer, coral, "eject")).withTimeout(0.2)
      .andThen(new DefaultPosition(_scorer, _elevator, coral))
    );
    NamedCommands.registerCommand("score coral L2", new SetupToScoreReef(_elevator, _scorer, 2)
      .andThen(new ManipulateGamePiece(_scorer, coral, "eject")).withTimeout(0.2)
      .andThen(new DefaultPosition(_scorer, _elevator, coral))
    );
    NamedCommands.registerCommand("score coral L3", new SetupToScoreReef(_elevator, _scorer, 3)
      .andThen(new ManipulateGamePiece(_scorer, coral, "eject")).withTimeout(0.2)
      .andThen(new DefaultPosition(_scorer, _elevator, coral))
    );
    NamedCommands.registerCommand("score coral L4", new SetupToScoreReef(_elevator, _scorer, 4)
      .andThen(new ManipulateGamePiece(_scorer, coral, "eject")).withTimeout(0.2)
      .andThen(new DefaultPosition(_scorer, _elevator, coral))
    );

    // Button board configuration 
    configureBindings();

    // smartdashboard dropdown
    autoChooserInit();
    
    // Smartdashboard dropdown 
    // error because doesn't exist lol
    
    // Resetting encoders
    _drive.syncEncoders();
    
    // Logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  public void startTeleop() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      _drive.flipGyro();
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
    _drive.setDefaultCommand(
      new DriveWithJoystick(_drive, m_driverController)    
    );

    // Syncs encoders   
    m_driverController.b().onTrue(
      Commands.runOnce(() -> _drive.syncEncoders(), _drive) 
    );

    m_driverController.y().onTrue(new ResetGyro(_drive));

    Command driveToRightTag = new DriveToTag(_drive, _rightCamera, VisionConstants.centeredDeltaX, VisionConstants.centeredDeltaY);
    m_driverController.rightBumper().whileTrue(driveToRightTag);

    Command driveToLeftTag = new DriveToTag(_drive, _leftCamera, VisionConstants.centeredDeltaX, VisionConstants.centeredDeltaY);
    m_driverController.leftBumper().whileTrue(driveToLeftTag);

    // In order to properly run characterization tests, it is best to be able to
    // manually control the stop/stop of the logger to remove as much noise.
    m_driverController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    m_driverController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    _elevator.setDefaultCommand(new OperateElevatorWithJoystick(_elevator, m_functionController));
    _scorer.setDefaultCommand(new OperateScorerWithJoystick(_scorer, m_functionController));
  }

  public void autoChooserInit() {
    String[] autoselector = {
      "drive 3m",
      "c4,b4,a4",
      "d4,c4,b4",
      "e4,d4,c4",
      "f4,e4,d4"
    };
    SmartDashboard.putStringArray("Auto List", autoselector);
    System.out.print("Loading selections");
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String selection = SmartDashboard.getString("Auto Selector", "None");
    
    Command autoCommand = Commands.runOnce(
      () -> _drive.freezeWheels(), _drive
    );  //The default command will be to freeze if nothing is selected

    autoCommand = new PathPlannerAuto(selection);
    return autoCommand;
  }
}
