// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.undo.StateEdit;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants;
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
    public static final double wheelDiameter = 3.83;
    public static final double wheelCircumference = wheelDiameter*Math.PI;
    public static final double maxAttainableSpeed = 4.2; // arbitrary right now
    public static final double maxSlowDriveSpeed = 2; //meters per second
    public static final double maxFastDriveSpeed = 4;
    public static final double maxDriveAcceleration = 1; //meters per second squared
    public static final double maxRotationSpeed = 1.5 * Math.PI; //radians per second
    public static final double maxRotationAcceleration = 3 * Math.PI; // guessed 2x max speed from PPLib examples
    public static final double maxSlewRate = 20;
    public static final double robotLength = .7; // meters

    public static final double driveGearRatio = 6.54;
    public static final double steerGearRatio = 11.3142;

    public static double trackWidth = Units.inchesToMeters(25);
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
    
    public static final double topLeftEncoderOffset = 4.338;
    public static final double topRightEncoderOffset = 2.928;
    public static final double bottomLeftEncoderOffset = 4.988;
    public static final double bottomRightEncoderOffset = 5.859; 

    public static KrakenSwerveModule[] getSwerveModules() {
      return new KrakenSwerveModule[] {
        new KrakenSwerveModule(3, 4, 0, topLeftEncoderOffset, "top left"), 
        new KrakenSwerveModule(5, 6, 1, topRightEncoderOffset, "top right"),
        new KrakenSwerveModule(1, 2, 2, bottomLeftEncoderOffset, "bottom left"),
        new KrakenSwerveModule(7, 8, 3, bottomRightEncoderOffset, "bottom right")
      };
    }

    public static final double turningP = 20;
    public static final double turningI = 0;
    public static final double turningD = 0;
    public static final double turningPIDOutputMin = -1;
    public static final double turningPIDOutputMax = 1;

    public static final double drivingP = 0;
    public static final double drivingI = 0; 
    public static final double drivingD = 0;
    public static final double drivingFF = 0.76;
    public static final double drivingPIDOutputMin = -1;
    public static final double drivingPIDOutputMax = 1;

    public static final double autoDriveP = 0.7;
    public static final double autoDriveI = 0; 
    public static final double autoDriveD = 0;
    public static final double autoTurningP = 0.8;
    public static final double autoTurningI = 0; 
    public static final double autoTurningD = 0;

    public static final double headingP = 0.11; 
    public static final double headingI = 0.02; 
    public static final double headingD = 0;
    public static final double headingPeriod = .02;

    public static final PIDController headingPIDController = new PIDController(headingP, headingI, headingD);

    public static final double driveCurrentThreshold = 50.0;
    public static final double driveTimeThreshold = 0.1;
    public static final double steerCurrentThreshold = 50.0;
    public static final double steerTimeThreshold = 0.1;

    public static final HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      new PIDConstants(autoDriveP, autoDriveI,autoDriveD), // Translation PID constants
      new PIDConstants(autoTurningP, autoTurningI, autoTurningI), // Rotation PID constants
      RobotConstants.maxAttainableSpeed, // Max module speed, in m/s
      RobotConstants.robotDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig(true, true) // can add thresholds for both values, see PPLib Java API for info
    );
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
