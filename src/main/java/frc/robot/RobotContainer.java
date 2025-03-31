// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
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
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DriveToTag.CameraSide;
import frc.robot.commands.DriveToTag.ReefSide;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.DriveWithSpeed;
import frc.robot.commands.EjectCoralUntilOut;
import frc.robot.commands.ElevatorPastHeight;
import frc.robot.commands.ElevatorToHeight;
import frc.robot.commands.IntakeCoralUntilIn;
import frc.robot.commands.ManipulateGamePiece;
import frc.robot.commands.OperateClimberWithJoystick;
import frc.robot.commands.OperateElevatorWithJoystick;
import frc.robot.commands.OperateScorerWithJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.RotateClimber;
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
  private final Vision _vision = new Vision();
  private final Elevator _elevator = new Elevator();
  private final Drive _drive = new Drive(_elevator);
  private final Scorer _scorer = new Scorer();
  private final Climb _climb = new Climb();

  private HashMap<String, Command> _autoMap = new HashMap<String, Command>();

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
    NamedCommands.registerCommand("drive to left tag", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.any));
    NamedCommands.registerCommand("drive to right tag", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.any));

    NamedCommands.registerCommand("drive to A", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.ab));
    NamedCommands.registerCommand("drive to B", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.ab));
    NamedCommands.registerCommand("drive to C", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.cd));
    NamedCommands.registerCommand("drive to D", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.cd));
    NamedCommands.registerCommand("drive to E", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.ef));
    NamedCommands.registerCommand("drive to F", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.ef));
    NamedCommands.registerCommand("drive to G", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.gh));
    NamedCommands.registerCommand("drive to H", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.gh));
    NamedCommands.registerCommand("drive to I", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.ij));
    NamedCommands.registerCommand("drive to J", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.ij));
    NamedCommands.registerCommand("drive to K", new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.kl));
    NamedCommands.registerCommand("drive to L", new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.kl));

    NamedCommands.registerCommand("drop funnel", new RotateClimber(_climb, ClimbConstants.lowerFunnel)
    .andThen(new RotateClimber(_climb, ClimbConstants.lockFunnel)));

    NamedCommands.registerCommand("default coral position", 
        new RotateScorer(_scorer, ScorerConstants.rotationL2).withTimeout(.5)
          .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralDefault))
          .andThen(new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault)));
    NamedCommands.registerCommand("prep to score L2", 
        new RotateScorer(_scorer, ScorerConstants.rotationL2).withTimeout(.5)
          .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL2)));
    NamedCommands.registerCommand("prep to score L3", 
        new RotateScorer(_scorer, ScorerConstants.rotationL2).withTimeout(.5)
          .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL3))
          .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL3)));
    NamedCommands.registerCommand("prep to score L4", 
        new RotateScorer(_scorer, ScorerConstants.rotationL2).withTimeout(.5)
          .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL4))
          .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL4)));

    NamedCommands.registerCommand("elevate to L4", 
          new RotateScorer(_scorer, ScorerConstants.rotationL2).withTimeout(.5)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL4)));

    NamedCommands.registerCommand("slow drive forwards", new DriveWithSpeed(_drive, 0.2));
    NamedCommands.registerCommand("slow drive backwards", new DriveWithSpeed(_drive, -0.2));

    NamedCommands.registerCommand("intake coral", new IntakeCoralUntilIn(_scorer).withTimeout(2));
    NamedCommands.registerCommand("eject coral", new EjectCoralUntilOut(_scorer));

    // Button board configuration 
    configureBindings();

    autoChooserInit();
    
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
    BooleanSupplier sillyClimbMode = () -> !buttonBoard2[6].getAsBoolean();
    BooleanSupplier overrideStops = () -> buttonBoard2[3].getAsBoolean();

    // prep to score buttons
    // l4
    buttonBoard1[6].onTrue(
      Commands.either(
        new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault)
            .andThen(new ElevatorPastHeight(_elevator, ElevatorConstants.heightCoralL4))
            .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL4)
            .alongWith(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL4))),
        // algae mode
        new InstantCommand(),
        gamepieceMode)
    );
    // l3
    buttonBoard1[5].onTrue(
      Commands.either(
        // coral mode
        new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL3))
            .andThen(new RotateScorer(_scorer, ScorerConstants.rotationL3)),
        // algae mode
        new RotateScorer(_scorer, ScorerConstants.rotationAlgaeTop).withTimeout(1)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeTop)), 
        gamepieceMode)
    );
    // l2
    buttonBoard1[8].onTrue(
      Commands.either(
        // coral mode
        new RotateScorer(_scorer, ScorerConstants.rotationL2)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL2)),
        // algae mode
        new RotateScorer(_scorer, ScorerConstants.rotationAlgaeBottom).withTimeout(1)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeBottom)), 
        gamepieceMode).withTimeout(7)
    );
    // default position
    buttonBoard1[4].onTrue(
      Commands.either(
        // coral mode
        new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault)
        .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralDefault))
        .andThen(new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault)),
        // algae mode
        new RotateScorer(_scorer, ScorerConstants.rotationAlgaeProcessor).withTimeout(1)
            .andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeProcessor)), 
        gamepieceMode)
    );

    buttonBoard1[7].onTrue(
      Commands.either(
        // coral mode
        new ElevatorToHeight(_elevator, 20)
        .andThen(new RotateScorer(_scorer, 16.6))
        .andThen(new ManipulateGamePiece(_scorer, gamepieceMode, true)).withTimeout(1.3)
        .andThen(new ElevatorToHeight(_elevator, 30).alongWith(new ManipulateGamePiece(_scorer, gamepieceMode, true).withTimeout(.4)))
        .andThen(new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL1)))
        ,
        // algae mode
        new InstantCommand(), 
        gamepieceMode)
    );
  
    buttonBoard2[7].onTrue(new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralL1)));
    buttonBoard2[7].onFalse(new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeDefault).andThen(new RotateScorer(_scorer, ScorerConstants.rotationAlgaeDefault)));

    buttonBoard2[6].onTrue(Commands.runOnce(() -> _drive.disableYawLock(), _drive));
    buttonBoard2[6].onFalse(Commands.runOnce(() -> _drive.enableYawLock(), _drive));
    
    // BooleanSupplier noHeightPressed = () -> !(buttonBoard1[6].getAsBoolean() || buttonBoard1[5].getAsBoolean() || buttonBoard1[8].getAsBoolean());
    // Trigger noHeightPressedTrigger = new Trigger(noHeightPressed);

    // noHeightPressedTrigger.onTrue(Commands.either(
    //   new RotateScorer(_scorer, ScorerConstants.rotationCoralDefault).andThen(new ElevatorToHeight(_elevator, ElevatorConstants.heightCoralDefault)), 
    //   new ElevatorToHeight(_elevator, ElevatorConstants.heightAlgaeDefault).andThen(new RotateScorer(_scorer, ScorerConstants.rotationAlgaeDefault)),
    //   gamepieceMode));

    // zero elevator encoder
    buttonBoard2[2].onTrue(Commands.either(
      Commands.runOnce(() -> _elevator.syncEncoders(), _elevator),
      Commands.runOnce(() -> _climb.syncEncoder(), _climb),
      sillyClimbMode
      )
    );

    buttonBoard1[3].onTrue(new RotateClimber(_climb, ClimbConstants.lowerFunnel).andThen(new RotateClimber(_climb, ClimbConstants.lockFunnel)));

    // zero scorer rotation encoder
    buttonBoard2[1].onTrue(
      Commands.runOnce(() -> _scorer.syncRotationEncoder(), _scorer) 
    );
    _elevator.setDefaultCommand(new OperateElevatorWithJoystick(_elevator, _functionJoystick1, climbMode, overrideStops));
    _scorer.setDefaultCommand(new OperateScorerWithJoystick(_scorer, _functionJoystick1, overrideStops));
    _climb.setDefaultCommand(new OperateClimberWithJoystick(_climb, _functionJoystick1, climbMode, overrideStops));

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

    Command driveToRightTag = new DriveToTag(_drive, _vision, CameraSide.RIGHT, ReefSide.any);
    m_driverController.rightBumper().whileTrue(driveToRightTag);

    Command driveToLeftTag = new DriveToTag(_drive, _vision, CameraSide.LEFT, ReefSide.any);
    m_driverController.leftBumper().whileTrue(driveToLeftTag);

    // _scorer.setDefaultCommand(new OperateScorerWithJoystick(_scorer, m_functionController));
    // _elevator.setDefaultCommand(new OperateElevatorWithJoystick(_elevator, m_functionController));
  }

  public void autoChooserInit() {
    String[] autoselector = {
      "f4, d4",
      "i4, k4",
      "i4, k4, push",
      "h4"
    };
    SmartDashboard.putStringArray("Auto List", autoselector);
    System.out.print("Loading selections");
    for(String auto : autoselector) {
      _autoMap.put(auto, new PathPlannerAuto(auto));
    }
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
    
    autoCommand = _autoMap.get(selection);
    return autoCommand;  
  }
}
