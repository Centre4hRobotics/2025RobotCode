// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.RotateClimber;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.DriveWithSpeed;
import frc.robot.commands.EjectCoralUntilOut;
import frc.robot.commands.ElevatorToHeight;
import frc.robot.commands.IntakeCoralUntilIn;
import frc.robot.commands.ManipulateGamePiece;
import frc.robot.commands.OperateClimberWithJoystick;
import frc.robot.commands.OperateElevatorWithJoystick;
import frc.robot.commands.OperateScorerWithJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RotateScorer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Scorer;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here..-
  private final Vision _vision = new Vision("RIGHT");
  private final Elevator _elevator = new Elevator();
  private final Drive _drive = new Drive(_elevator);
  private final Scorer _scorer = new Scorer();
  private final Climb _climb = new Climb();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      private final Joystick _functionJoystick1 = new Joystick(1);
      private final Joystick _functionJoystick2 = new Joystick(2);

  //private final Joystick _functionJoystick = new Joystick(1);
  //private final Joystick _functionJoystick2 = new Joystick(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // register pathplanner commands

    BooleanSupplier coral = () -> true;
    BooleanSupplier station = () -> true;
    NamedCommands.registerCommand("drive to left tag", new DriveToTag(_drive, _vision, 0, VisionConstants.centeredDeltaX, "LEFT"));
    NamedCommands.registerCommand("drive to right tag", new DriveToTag(_drive, _vision, 0, VisionConstants.centeredDeltaX, "RIGHT"));

    NamedCommands.registerCommand("default coral position", new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault)
    .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL1)));
    NamedCommands.registerCommand("prep to score L2", new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL2)
    .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL2)));
    NamedCommands.registerCommand("prep to score L3", new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL3)
    .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL3)));
    NamedCommands.registerCommand("prep to score L4", new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL4)
    .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL4)));

    NamedCommands.registerCommand("elevate to L2", new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL2));
    NamedCommands.registerCommand("elevate to L3", new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL3));
    NamedCommands.registerCommand("elevate to L4", new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL4));

    NamedCommands.registerCommand("slow drive forwards", new DriveWithSpeed(_drive, 0.2));
    NamedCommands.registerCommand("slow drive backwards", new DriveWithSpeed(_drive, -0.2));

    NamedCommands.registerCommand("intake coral", new IntakeCoralUntilIn(_scorer).withTimeout(2));
    NamedCommands.registerCommand("eject coral", new EjectCoralUntilOut(_scorer));
    

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
    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //   _drive.flipGyro();
    // }
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

     // size is up to max id, not number of buttons
     JoystickButton[] buttonBoard1 = new JoystickButton[13];
     JoystickButton[] buttonBoard2 = new JoystickButton[8];

     for (int i = 1; i <= 12; i++) {
      buttonBoard1[i] = new JoystickButton(_functionJoystick1, i);
    }

    for (int i = 1; i <= 4; i++) {
      buttonBoard2[i] = new JoystickButton(_functionJoystick2, i);
    }
    for (int i = 6; i <= 7; i++) {
      buttonBoard2[i] = new JoystickButton(_functionJoystick2, i);
    }

    BooleanSupplier gamepieceMode = buttonBoard2[7];
    BooleanSupplier climbMode = buttonBoard2[6];
    BooleanSupplier overrideStops = () -> buttonBoard2[3].getAsBoolean();

    // prep to score buttons
    buttonBoard1[6].whileTrue(
      Commands.either(
        // coral mode
        new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL4)
            .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL4)),
        // algae mode
        new InstantCommand(),
        gamepieceMode
      )
    );
    buttonBoard1[5].whileTrue(
      Commands.either(
        // coral mode
        new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL3)
            .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL3)),
        // algae mode
        new RotateScorer(_scorer, ScorerConstants.rotationAlgaeTop)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeTop)), 
        gamepieceMode)
    );
    buttonBoard1[8].whileTrue(
      Commands.either(
        // coral mode
        new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL2)
            .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL2)),
        // algae mode
        new RotateScorer(_scorer, ScorerConstants.rotationAlgaeBottom)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeBottom)), 
        gamepieceMode)
    );
    buttonBoard1[4].whileTrue(
      Commands.either(
        // coral mode
        new RotateScorer(_scorer, ScorerConstants.rotationL1)
        .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL1)),
        // algae mode
        new RotateScorer(_scorer, ScorerConstants.rotationAlgaeProcessor)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeProcessor)), 
        gamepieceMode)
    );
  
    buttonBoard2[7].onTrue(new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL1)));
    buttonBoard2[7].onFalse(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeDefault).andThen(new RotateScorer(_scorer, ScorerConstants.rotationAlgaeDefault)));
    
    // if any other height buttons are pressed when one is released, it will not go to default position so it stays at the position being held
    BooleanSupplier heightButtonPressed = () -> buttonBoard1[6].getAsBoolean() || buttonBoard1[5].getAsBoolean() || buttonBoard1[8].getAsBoolean();

    buttonBoard1[6].onFalse(
      Commands.either(new InstantCommand(), Commands.either(
        new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralDefault)), 
        new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeDefault).andThen(new RotateScorer(_scorer, ScorerConstants.rotationAlgaeDefault)),
        gamepieceMode), heightButtonPressed));
    buttonBoard1[5].onFalse(
      Commands.either(new InstantCommand(), Commands.either(
        new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralDefault)), 
        new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeDefault).andThen(new RotateScorer(_scorer, ScorerConstants.rotationAlgaeDefault)),
        gamepieceMode), heightButtonPressed));
    buttonBoard1[8].onFalse(
      Commands.either(new InstantCommand(), Commands.either(
        new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralDefault)), 
        new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeDefault).andThen(new RotateScorer(_scorer, ScorerConstants.rotationAlgaeDefault)),
        gamepieceMode), heightButtonPressed));

    // zero elevator encoder
    buttonBoard2[2].onTrue(
      Commands.runOnce(() -> _elevator.syncEncoders(), _elevator) 
    );

    buttonBoard1[3].whileTrue(new DriveWithSpeed(_drive, 1));

    // zero scorer rotation encoder
    buttonBoard2[1].onTrue(
      Commands.runOnce(() -> _scorer.syncRotationEncoder(), _scorer) 
    );
    _elevator.setDefaultCommand(new OperateElevatorWithJoystick(_elevator, _functionJoystick1, overrideStops));
    _scorer.setDefaultCommand(new OperateScorerWithJoystick(_scorer, _functionJoystick1, climbMode, overrideStops));
    _climb.setDefaultCommand(new OperateClimberWithJoystick(_climb, _functionJoystick1, climbMode));

    // runs wheels on scorer
    buttonBoard1[10].whileTrue(new ManipulateGamePiece(_scorer, gamepieceMode, true));
    buttonBoard2[4].whileTrue(new ManipulateGamePiece(_scorer, gamepieceMode, false));
    buttonBoard1[11].whileTrue(new IntakeCoralUntilIn(_scorer));
    buttonBoard1[9].whileTrue(new EjectCoralUntilOut(_scorer));
    buttonBoard1[12].whileTrue(new ManipulateGamePiece(_scorer, gamepieceMode, true, true));

    buttonBoard1[1].onTrue(new RotateClimber(_climb, ClimbConstants.dropFunnel).andThen(new RotateClimber(_climb, ClimbConstants.pushFunnel)).andThen(new RotateClimber(_climb, ClimbConstants.prepClimb)));
    buttonBoard1[2].onTrue(new RotateClimber(_climb, ClimbConstants.climbed));
  
    // Syncs encoders   
    // m_driverController.b().onTrue(
    //   Commands.runOnce(() -> _drive.syncEncoders(), _drive) 
    // );

    m_driverController.y().onTrue(new ResetGyro(_drive));

    Command driveToRightTag = new DriveToTag(_drive, _vision, VisionConstants.centeredDeltaX, VisionConstants.centeredDeltaY, "RIGHT");
    m_driverController.rightBumper().whileTrue(driveToRightTag);

    Command driveToLeftTag = new DriveToTag(_drive, _vision, VisionConstants.centeredDeltaX, VisionConstants.centeredDeltaY, "LEFT");
    m_driverController.leftBumper().whileTrue(driveToLeftTag);

    // _scorer.setDefaultCommand(new OperateScorerWithJoystick(_scorer, m_functionController));
    // _elevator.setDefaultCommand(new OperateElevatorWithJoystick(_elevator, m_functionController));
  }

  public void autoChooserInit() {
    String[] autoselector = {
      "e4, c4",
      "f4, e4",
      "g4",
      "j4, l4"
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
