// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.KrakenSwerveModule;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kFunctionControllerPort = 1;
  }

  // implement everywhere else
  public static enum RobotType {
    PRACTICE,
    COMPETITION
  }

  public static RobotType robotType = RobotType.COMPETITION;

  public static class RobotConstants {
    public static final double wheelDiameter;

    static{
      switch(robotType)
      {
        case COMPETITION:
          wheelDiameter = 3.89;
          break;
        case PRACTICE:
        default:
          wheelDiameter = 4.0;
          break;
      }
    }

    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double maxAttainableSpeed; // arbitrary right now
    public static final double maxSlowDriveSpeed; //meters per second
    public static final double maxFastDriveSpeed;
    public static final double maxDriveAcceleration; //meters per second squared
    public static final double maxRotationSpeed; //radians per second
    public static final double maxRotationAcceleration; // guessed 2x max speed from PPLib examples

    public static final double driveGearRatio;
    public static final double steerGearRatio;
    static {
      switch (robotType)
      {
        case COMPETITION:
          maxAttainableSpeed = 4.2;
          maxSlowDriveSpeed = 1.5;
          maxFastDriveSpeed = 4;
          maxDriveAcceleration = 1;
          maxRotationSpeed = 1.5 * Math.PI;
          maxRotationAcceleration = 3 * Math.PI;
          driveGearRatio = 6.2;
          steerGearRatio = 12.1;
          break;
        case PRACTICE:
        default:
          maxAttainableSpeed = 4.2;
          maxSlowDriveSpeed = 1.5;
          maxFastDriveSpeed = 0.7;
          maxDriveAcceleration = 1;
          maxRotationSpeed = 1.5 * Math.PI;
          maxRotationAcceleration = 3 * Math.PI;
          driveGearRatio = 6.54;
          steerGearRatio = 11.3142;
          break;
      }
    }

    public static final double maxSlewRate = 20;
    public static final double robotLength = .7; // meters

    public static double trackWidth;

    static{
      switch(robotType)
      {
        case COMPETITION:
          trackWidth = Units.inchesToMeters(25);
          break;
        case PRACTICE:
        default:
          trackWidth = Units.inchesToMeters(23);
          break;
      }
    }

    public static final double robotDriveBaseRadius = trackWidth / Math.sqrt(2); //distance from center to swerve module

    // top left, top right, bottom left, bottom right
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      new Translation2d(trackWidth / 2, trackWidth / 2), // ++
      new Translation2d(trackWidth / 2, -trackWidth / 2), // +-
      new Translation2d(-trackWidth / 2, trackWidth / 2), // -+
      new Translation2d(-trackWidth / 2, -trackWidth / 2) // --
    );
  }

  public static class GyroConstants{
    public static final int pigeonID = 11;

    public static final double mountPoseYawDegrees = 0;
    public static final double mountPosePitchDegrees = 0;
    public static final double mountPoseRollDegrees = 0;
  }

  public static class MotorConstants{
    
    public static final double topLeftEncoderOffset;
    public static final double topRightEncoderOffset;
    public static final double bottomLeftEncoderOffset;
    public static final double bottomRightEncoderOffset;
    public static final InvertedValue inverted;
    
    static {
      switch (robotType)
      {
        case COMPETITION:
          bottomLeftEncoderOffset = 6.23;
          bottomRightEncoderOffset = 3.51;
          topLeftEncoderOffset = 2.13;
          topRightEncoderOffset = 3.14;
          inverted = InvertedValue.Clockwise_Positive;
          break;
        case PRACTICE:
        default:
          topLeftEncoderOffset = 4.338;
          topRightEncoderOffset = 2.928;
          bottomLeftEncoderOffset = 4.988;
          bottomRightEncoderOffset = 5.859;
          inverted = InvertedValue.CounterClockwise_Positive;
          break;
      }
    }

    public static KrakenSwerveModule[] getSwerveModules() {
      return new KrakenSwerveModule[] {
        new KrakenSwerveModule(3, 4, 0, topLeftEncoderOffset, "top left"), 
        new KrakenSwerveModule(5, 6, 1, topRightEncoderOffset, "top right"),
        new KrakenSwerveModule(1, 2, 2, bottomLeftEncoderOffset, "bottom left"),
        new KrakenSwerveModule(7, 8, 3, bottomRightEncoderOffset, "bottom right")
      };
    }

    // Turning refers to the wheels themselves in the swerve module.
    public static final double wheelTurningP;
    public static final double wheelTurningI;
    public static final double wheelTurningD;

    // Driving refers to teleoperate mode and the linear translation of the robot.
    public static final double drivingP;
    public static final double drivingI; 
    public static final double drivingD;
    public static final double drivingS;
    public static final double drivingFF;

    // Auto refers to the autonomous mode (contains both drive and wheel).
    public static final double autoDriveP;
    public static final double autoDriveI; 
    public static final double autoDriveD;
    public static final double autoTurningP;
    public static final double autoTurningI; 
    public static final double autoTurningD;

    // Heading refers to the heading of the entire robot as a whole.
    public static final double headingP; 
    public static final double headingI; 
    public static final double headingD;
    public static final double headingPeriod;

    // Creates the current/time thresholds
    // Current limits the draw of current
    // Time limits how long a motor can stall before decreasing power draw
    public static final double driveCurrentThreshold;
    public static final double driveTimeThreshold;
    public static final double steerCurrentThreshold;
    public static final double steerTimeThreshold;

    static {
      switch (robotType)
      {
        case COMPETITION:
          wheelTurningP = 20;
          wheelTurningI = 0;
          wheelTurningD = 0;

          drivingP = 0;
          drivingI = 0; 
          drivingD = 0;
          drivingS = 0.9;
          drivingFF = 0.73;

          autoDriveP = 0.7;
          autoDriveI = 0; 
          autoDriveD = 0;
          autoTurningP = 0.8;
          autoTurningI = 0; 
          autoTurningD = 0;

          headingP = 0.11; 
          headingI = 0.02; 
          headingD = 0;
          headingPeriod = .02;

          driveCurrentThreshold = 50.0;
          driveTimeThreshold = 0.1;
          steerCurrentThreshold = 50.0;
          steerTimeThreshold = 0.1;
          break;
        case PRACTICE:
        default:
          wheelTurningP = 19.526;
          wheelTurningI = 0;
          wheelTurningD = 0.27154;

          drivingP = 0.1;
          drivingI = 0; 
          drivingD = 0;
          drivingS = 0.9;
          drivingFF = 0.76;

          autoDriveP = 0.7;
          autoDriveI = 0; 
          autoDriveD = 0;

          autoTurningP = 0.4;
          autoTurningI = 0; 
          autoTurningD = 0;

          headingP = 0.11; 
          headingI = 0.02; 
          headingD = 0;
          headingPeriod = .02;

          driveCurrentThreshold = 50.0;
          driveTimeThreshold = 0.1;
          steerCurrentThreshold = 50.0;
          steerTimeThreshold = 0.1;
          break;
      }
    }

    public static final PIDController headingPIDController = new PIDController(headingP, headingI, headingD);


    // HolonomicPathFollowerConfig, this should likely live in your Constants class
    // public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig( 
    //   new PIDConstants(autoDriveP, autoDriveI,autoDriveD), // Translation PID constants
    //   new PIDConstants(autoTurningP, autoTurningI, autoTurningI), // Rotation PID constants
    //   RobotConstants.maxAttainableSpeed, // Max module speed, in m/s
    //   RobotConstants.robotDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
    //   new ReplanningConfig(true, true) // can add thresholds for both values, see PPLib Java API for info
    // );
  }

  
  public static class VisionConstants
  {
    public static final double cameraRoll = 0.0;
    public static final double cameraPitch = 0.0;
    public static final double cameraYaw = 0.0;

    public static final double distanceTolerance = 0.05;
    public static final double rotationTolerance = 0.05;

    public static final double centeredDeltaX = RobotConstants.trackWidth/2.0;
    // Positive offset is to the right
    public static final double centeredDeltaY = 0;
    public static final double laserDistanceToReef = 0.3;

    // Tag refers to the april tag positioning system
    public static final double tagDriveXP;
    public static final double tagDriveXI; 
    public static final double tagDriveXD;

    public static final double tagDriveYP;
    public static final double tagDriveYI; 
    public static final double tagDriveYD;

    public static final double tagTurningP;
    public static final double tagTurningI; 
    public static final double tagTurningD;

    static {
      switch (robotType)
      {
        case COMPETITION:
          // Forwards
          tagDriveXP = 0.6; //.55
          tagDriveXI = 0.03; 
          tagDriveXD = 0;
          // Side
          tagDriveYP = 2; 
          tagDriveYI = 0.0; 
          tagDriveYD = 0.02; 
          // Turning
          tagTurningP = 2; //.1
          tagTurningI = 0.0; 
          tagTurningD = 0.0;
          break;
        case PRACTICE:
        default:
          // Forwards
          tagDriveXP = 1; //.55
          tagDriveXI = 0.03; 
          tagDriveXD = 0;
          // Side
          tagDriveYP = 3; 
          tagDriveYI = 0.0; 
          tagDriveYD = 0.02; 
          // Turning
          tagTurningP = 2; //.1
          tagTurningI = 0.0; 
          tagTurningD = 0.0;
          break;
      }
    }

  }

  public static class ElevatorConstants {
    // coral
    public static double heightCoralL1 = 0; 
    public static double heightCoralL2 = 18.3;
    public static double heightCoralL3 = 43.4;
    public static double heightCoralL4 = 83; 

    public static double heightAlgaeProcessor = 20; 
    public static double heightAlgaeOnCoral = 18.4; 
    public static double heightAlgaeDefault = 47;
    public static double heightAlgaeBottom = 47; 
    public static double heightAlgaeTop = 71; 
    public static double heightAlgaeBarge = 40;

    public static final double maxHeight = 85;
    public static final double maxFullSpeedHeight = 20;
    public static final double maxHeightVelocityPercent = 0.1;

    public static final double elevatorP = 3.5;

    public static final double elevatorI = 0;
    public static final double elevatorD = 0;

    public static final double leadCurrentThreshold = 25;
    public static final double followCurrentThreshold = 25;

    public static final double heightTolerance = 0.2;
  }

  public static class ScorerConstants {
    //coral
    public static double rotationCoralDefault = 0; 
    public static double rotationL1 = 0; 
    public static double rotationL2 = 2; 
    public static double rotationL3 = 0;
    public static double rotationL4 = 5.9; 

    // algae
    public static double rotationAlgaeTop = 36.5; 
    public static double rotationAlgaeOnCoral = 34; 
    public static double rotationAlgaeBottom = 36.5;
    public static double rotationAlgaeDefault = 36.5; 
    public static double rotationAlgaeProcessor = 40;
    public static double rotationAlgaeBarge = 40; //this is a meme

    public static final double rotationP = 0.16;
    public static final double rotationI = 0;
    public static final double rotationD = 0;

    public static final int rotationCurrentThreshold = 30;
    public static final int scoringCurrentThreshold = 40;
    public static final double scoringCurrentWithCoral = 20;

    public static final double numRotationsToIntake = 0.22;
    public static final double numRotationsToEject = 15;
    
    public static final double intakingCoralVoltage = -1.0;
    public static final double ejectingCoralVoltage = -1.0;
    public static final double backdrivingCoralVoltage = 3.0;
    public static final double stuckCoralVoltage = 8.0;

    public static final double intakingAlgaeVoltage = -6.0;
    public static final double ejectingAlgaeVoltage = 6.0;
    public static final double holdingAlageVoltage = -1.0;

    public static final double rotationTolerance = 1; //add

    public static final double lowestElevatingRotationCoral = 0; //add
    public static final double highestElevatingRotationCoral = 0; //add
    public static final double lowestElevatingRotationAlgae = 0; //add
    public static final double highestElevatingRotationAlgae = 0; //add
  }

  public static class FunnelConstants {
    public static final double heightStation = 0;
    public static final double heightClimb = 0;

    public static final double funnelP = 0;
    public static final double funnelI = 0;
    public static final double funnelD = 0;
  }

  public static class ClimbConstants {
    public static final int climbingCurrentThreshold = 30;

    public static final double climbingP = 1.0;
    public static final double climbingI = 0.0;
    public static final double climbingD = 0.0;

    public static final double climbRotationTolerance = 5.0;
  }

  public static enum FieldSide {
    LEFT, RIGHT
  }
}
