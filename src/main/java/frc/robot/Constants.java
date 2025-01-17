// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.undo.StateEdit;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  }

  // implement everywhere else
  public static enum RobotType {
    PRACTICE,
    COMPETITION
  }

  public static RobotType robotType = RobotType.PRACTICE;

  public static class RobotConstants {
    public static final double wheelDiameter;

    static{
      switch(robotType)
      {
        case COMPETITION:
          wheelDiameter = 3.83;
          break;
        case PRACTICE:
        default:
          wheelDiameter = 3.83;
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

    static {
      switch (robotType)
      {
        case COMPETITION:
          maxAttainableSpeed = 4.2;
          maxSlowDriveSpeed = 2;
          maxFastDriveSpeed = .7;
          maxDriveAcceleration = 1;
          maxRotationSpeed = 1.5 * Math.PI;
          maxRotationAcceleration = 3 * Math.PI;
          break;
        case PRACTICE:
        default:
          maxAttainableSpeed = 4.2;
          maxSlowDriveSpeed = 2;
          maxFastDriveSpeed = .7;
          maxDriveAcceleration = 1;
          maxRotationSpeed = 1.5 * Math.PI;
          maxRotationAcceleration = 3 * Math.PI;
          break;
      }
    }

    public static final double maxSlewRate = 20;
    public static final double robotLength = .7; // meters

    public static final double driveGearRatio = 6.54;
    public static final double steerGearRatio = 11.3142;

    public static double trackWidth;

    static{
      switch(robotType)
      {
        case COMPETITION:
          trackWidth = Units.inchesToMeters(25);
          break;
        case PRACTICE:
        default:
          trackWidth = Units.inchesToMeters(25);
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

  public static class MotorConstants{
    
    public static final double topLeftEncoderOffset;
    public static final double topRightEncoderOffset;
    public static final double bottomLeftEncoderOffset;
    public static final double bottomRightEncoderOffset;
    
    static {
      switch (robotType)
      {
        case COMPETITION:
          topLeftEncoderOffset = 4.338;
          topRightEncoderOffset = 2.928;
          bottomLeftEncoderOffset = 4.988;
          bottomRightEncoderOffset = 5.859;
          break;
        case PRACTICE:
        default:
          topLeftEncoderOffset = 4.338;
          topRightEncoderOffset = 2.928;
          bottomLeftEncoderOffset = 4.988;
          bottomRightEncoderOffset = 5.859;
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

    // Tag refers to the april tag positioning system
    public static final double tagTurningP;
    public static final double tagTurningI; 
    public static final double tagTurningD;

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
          drivingFF = 0.76;

          autoDriveP = 0.7;
          autoDriveI = 0; 
          autoDriveD = 0;
          autoTurningP = 0.8;
          autoTurningI = 0; 
          autoTurningD = 0;

          tagTurningP = 0.8;
          tagTurningI = 0; 
          tagTurningD = 0;

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
          wheelTurningP = 20;
          wheelTurningI = 0;
          wheelTurningD = 0;

          drivingP = 0;
          drivingI = 0; 
          drivingD = 0;
          drivingS = 0.9;
          drivingFF = 0.76;

          autoDriveP = 0;
          autoDriveI = 0; 
          autoDriveD = 0;
          autoTurningP = 0;
          autoTurningI = 0; 
          autoTurningD = 0;

          tagTurningP = 0.8;
          tagTurningI = 0; 
          tagTurningD = 0;

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
    public static final PIDController tagHeadingPIDController = new PIDController(tagTurningP, tagTurningI, tagTurningD);


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
    // assuming X is forwards
    public static final double cameraOffsetX = 0;
    public static final double cameraOffsetY = 0.25;
    public static final double cameraOffsetZ = 0.3;

    public static final double cameraRoll = 0.0;
    public static final double cameraPitch = 0.0;
    public static final double cameraYaw = 0.0;
  }

  public static enum FieldSide {
    LEFT, RIGHT
  }
}
