// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.GyroConstants;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Drive extends SubsystemBase {

  // top left, top right, bottom left, bottom right
  // the encoder offsets are overrided later :T
  private SwerveModuleBase[] _swerveModules;
  private SysIdRoutine _sysIdRoutine;
  
  private Pigeon2 _gyro = new Pigeon2(GyroConstants.pigeonID);

  private SwerveDriveOdometry _odometry;

  private SwerveDrivePoseEstimator _poseEstimator;

  private double _desiredHeading;
  private PIDController _headingPIDController;
  private boolean _inYawLock = true;

  private SlewRateLimiter _slewRateLimiterX = new SlewRateLimiter(RobotConstants.maxSlewRate);
  private SlewRateLimiter _slewRateLimiterY = new SlewRateLimiter(RobotConstants.maxSlewRate);


  // for logging to advantagescope
  private final Field2d _field = new Field2d();

  // private HolonomicPathFollowerConfig _pathFollowerConfig;
  private RobotConfig _config;
  private SwerveDriveKinematics _kinematics;

  /** Creates a new DriveSubsystem. */
  public Drive() {

    _kinematics = RobotConstants.driveKinematics;
    
    try{
      _config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    _swerveModules = MotorConstants.getSwerveModules();
    _headingPIDController = MotorConstants.headingPIDController;
    

    _odometry = new SwerveDriveOdometry(
      _kinematics,
      Rotation2d.fromDegrees(getGyroAngle()),
      new SwerveModulePosition[] {
        _swerveModules[0].getPosition(),
        _swerveModules[1].getPosition(),
        _swerveModules[2].getPosition(),
        _swerveModules[3].getPosition()
      }
    );

    _headingPIDController.enableContinuousInput(0, 360);
    _desiredHeading = getGyroAngle();

    AutoBuilder.configure(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotRelativeSpeeds, 
      this::setDesiredRobotRelativeSpeeds, 
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(MotorConstants.autoDriveP, MotorConstants.autoDriveI, MotorConstants.autoDriveD), // Translation PID constants
                    new PIDConstants(MotorConstants.autoTurningP, MotorConstants.autoTurningI, MotorConstants.autoTurningD) // Rotation PID constants
            ),
      _config, 
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this
    );


    _poseEstimator = new SwerveDrivePoseEstimator(
      _kinematics,
      Rotation2d.fromDegrees(getGyroAngle()),
      getModulePositions(),
      _odometry.getPoseMeters()
    );

    resetGyroAngle();
    setDesiredHeading(0);
  
    _sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Units.Volts.of(4),
        null,
        (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> setDriveVoltages(volts.in(Units.Volts)),
         null,
         this
      )
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    _odometry.update(
      Rotation2d.fromDegrees(getGyroAngle()),
      getModulePositions()
    );

    _poseEstimator.update(
      Rotation2d.fromDegrees(getGyroAngle()),
      getModulePositions()
    );
    
    _field.setRobotPose(getPose());

    this.log();
  }

  /**
   * Scale inputs to velocities and use them as setpoints
   * @param rawX Either value between -1 and 1 or m/s
   * @param rawY Either value between -1 and 1 or m/s
   * @param rawAngular Either value between -1 and 1 or rad/s
   * @param scaleInput if true, assumes input is -1 to 1 and scales to max
   */
  public void drive(double rawX, double rawY, double rawAngular, double triggerSpeedupInput) {
    double angularVelocity = rawAngular;
    double xVelocity = rawX;
    double yVelocity = rawY;

      // scale raw values to max allowed
      angularVelocity *= RobotConstants.maxRotationSpeed;
      double triggerSpeedupValue = (RobotConstants.maxFastDriveSpeed - RobotConstants.maxSlowDriveSpeed) * triggerSpeedupInput;
      xVelocity *= RobotConstants.maxSlowDriveSpeed + triggerSpeedupValue;
      yVelocity *= RobotConstants.maxSlowDriveSpeed + triggerSpeedupValue;

    // in deadzone
    if (angularVelocity == 0) {
      // if not already locked (stick just released)
      if (!_inYawLock) {
        if(Math.abs(getGyroAngularVelocity()) < .5) {
          _inYawLock = true;
          _desiredHeading = getGyroAngle();
        }
      } else { 
        // yaw is locked
        angularVelocity = _headingPIDController.calculate(getGyroAngle(), _desiredHeading);
        // limit angular velocity to maxRotationSpeed
        // afterward scales with trigger input because it needs to be stronger at higher speeds
        // this is because of several things to do with max speed
        angularVelocity = Math.signum(angularVelocity) * Math.min(Math.abs(angularVelocity), RobotConstants.maxRotationSpeed) * (1+1.2*triggerSpeedupInput);
      }
    } else {
      _inYawLock = false;
    }

    if (Math.abs(xVelocity) < .02 && Math.abs(yVelocity) < .02 && Math.abs(angularVelocity) < .5) { 
      setWheelsO();
    } else {
      setDesiredStates(_slewRateLimiterX.calculate(xVelocity), _slewRateLimiterY.calculate(yVelocity), angularVelocity);
    } 

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("SwerveDrive").getEntry("desired angular velocity").setValue(angularVelocity);
    nt.getTable("SwerveDrive").getEntry("desired heading").setValue(_desiredHeading);
    nt.getTable("SwerveDrive").getEntry("gyro angle").setValue(getGyroAngle());
    nt.getTable("SwerveDrive").getEntry("angle difference").setValue(getGyroAngle() - _desiredHeading);
    nt.getTable("SwerveDrive").getEntry("gyro angular velocity").setValue(getGyroAngularVelocity());
  }
  
  /**
   * Override the desired heading 
   * @param heading Desired heading to point towards (in degrees)
   */
  public void setDesiredHeading(double heading) {
    _desiredHeading = heading; // + compensationAngle;
    _inYawLock = false;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return _kinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[_swerveModules.length];
    for (int i = 0; i < _swerveModules.length; i++) {
      states[i] = _swerveModules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      _swerveModules[0].getPosition(),
      _swerveModules[1].getPosition(),
      _swerveModules[2].getPosition(),
      _swerveModules[3].getPosition()
    };
  }

  public void setDesiredRobotRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds) {

    SwerveModuleState[] targetStates = _kinematics.toSwerveModuleStates(robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      targetStates, 
      RobotConstants.maxAttainableSpeed
    );
    setDesiredStates(targetStates);
  }

  /**
   * Sets the desired state for modules for PID to adjust to
   * @param xSpeed speed in x direction (field relative) in m/s?
   * @param ySpeed speed in y direction (field relative) in m/s?
   * @param angularVelocity angular velocity in rad/s
   */
  public void setDesiredStates(double xSpeed, double ySpeed, double angularVelocity) {
    ChassisSpeeds currentSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angularVelocity, Rotation2d.fromDegrees(getGyroAngle()));

    SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(
      // field relative
      currentSpeeds
      // not field relative
      // new ChassisSpeeds(xSpeed, ySpeed, angularVelocity)
    );  

    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, 
      RobotConstants.maxAttainableSpeed
    );

    for (int i = 0; i < _swerveModules.length; i++) {
      _swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  } 

  public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
    for (int i = 0; i < _swerveModules.length; i++) {
      _swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  /**
  * Sets the velocities for both the drive and rotation motors
  * @param driveSpeed Sets the velocity for the drive motor
  * @param angularSpeed Sets the velocity for the rotation motor
  */
  public void setSpeeds(double driveSpeed, double angularSpeed) {
    for(SwerveModuleBase swerveModule : _swerveModules) {
      swerveModule.setDriveSpeed(driveSpeed);
      swerveModule.setRotationSpeed(angularSpeed);
    }
  } 

  public double[] getDriveVelocities() {
    double[] driveVelocities = new double[_swerveModules.length];
    for(int i = 0; i < _swerveModules.length; i++) {
      driveVelocities[i] = _swerveModules[i].getDriveVelocity();
    }
    return driveVelocities;
  }

  public void setDriveVoltages(double voltage){
    for(SwerveModuleBase module : _swerveModules) {
      module.setDriveVoltage(voltage);
      module.setTurnEncoderPIDTarget(0.0);
    }

  }

  public void setTurnEncoderPIDTarget(double target) {
    for (SwerveModuleBase module : _swerveModules) {
      module.setTurnEncoderPIDTarget(target);
    }
  }

  /**
   * Point wheels in an X so robot doesn't move
   */
  public void setWheelsO() {
    _swerveModules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    _swerveModules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    _swerveModules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    _swerveModules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
  }

  /**
   * Point all wheels forward
   */
  public void setWheelsForward() {
    for (SwerveModuleBase swerveModule : _swerveModules) {
      swerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
    }
  }

  /**
   * Sets wheel velocities to 0 but keep wheel angles
   */
  public void freezeWheels() {
    for (SwerveModuleBase s : _swerveModules) {
      s.setDesiredState(new SwerveModuleState(0, s.getState().angle));
    }
  }

  /**
   * Get pose of robot in field in meters based on odometry
   * @return
   */
  public Pose2d getPose() {
    return _odometry.getPoseMeters();
    // return _poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPosePathPlanner() {
    return getPose().plus(new Transform2d(new Translation2d(0, 0), new Rotation2d(0.5 * Math.PI)));
  }

  public void resetOdometry(Pose2d pose) {
    resetGyroAngle(pose.getRotation().getDegrees());
    _odometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngle()),
      getModulePositions(),
      pose
    );
  }

  public void syncEncoders()
  {
    for(SwerveModuleBase module : _swerveModules)
        module.syncEncoder();
  }

  /**
   * Heading in degrees
   * @return
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(getGyroAngle()).getDegrees();
  }
  
  /**
   * Get gyro yaw (in degrees)
   * @return gyro angle
   */
  public double getGyroAngle() {
    return _gyro.getYaw().getValueAsDouble();
  }

  public double getGyroAngularVelocity() {
    return -_gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public double getGyroRoll() {
    return _gyro.getRoll().getValueAsDouble();
  }

  public void resetGyroAngle() {
    _gyro.reset();
  }

  public void resetGyroAngle(double angle) {
    _gyro.reset();
    _gyro.setYaw(-angle);
  }

  public void log() {
    // log overall states as list for AdvantageScope
    double[] states = new double[8];
    double[] desiredStates = new double[8];

    // get average velocity of wheels
    double averageVelocity = 0;
    for (int i = 0; i < 4; i++) {
      _swerveModules[i].log();
      averageVelocity += Math.abs(_swerveModules[i].getDriveVelocity()) / 4.;

      states[i * 2] = _swerveModules[i].getState().angle.getRadians() % Math.PI;
      if (states[i * 2] < 0) states[i * 2] += Math.PI; // so it's comparable in advantagescope
      states[i * 2 + 1] = Math.abs(_swerveModules[i].getState().speedMetersPerSecond);
      desiredStates[i * 2] = _swerveModules[i].getDesiredState().angle.getRadians() % Math.PI;
      if (desiredStates[i * 2] < 0) desiredStates[i * 2] += Math.PI;
      desiredStates[i * 2 + 1] = Math.abs(_swerveModules[i].getDesiredState().speedMetersPerSecond);
    }

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("DriveSubsystem").getEntry("Module States").setValue(states);
    nt.getTable("DriveSubsystem").getEntry("Desired Module States").setValue(desiredStates);
    nt.getTable("DriveSubsystem").getEntry("Gyro").setDouble(getGyroAngle());
    nt.getTable("DriveSubsystem").getEntry("Heading").setDouble(getHeading());
    nt.getTable("DriveSubsystem").getEntry("Desired Heading").setDouble(_desiredHeading);

    nt.getTable("DriveSubsystem").getEntry("Odometry").setValue(_odometry.getPoseMeters().toString());
    nt.getTable("DriveSubsystem").getEntry("Average Velocity").setValue(averageVelocity);

    // log robot pose as list
    Pose2d robotPose = _odometry.getPoseMeters();
    double[] robotPoseList = new double[3];
    robotPoseList[0] = robotPose.getX();
    robotPoseList[1] = robotPose.getY();
    robotPoseList[2] = robotPose.getRotation().getRadians();
    nt.getTable("DriveSubsystem").getEntry("Robot Pose").setValue(robotPoseList);

    // pose estimator 
    nt.getTable("DriveSubsystem").getEntry("Pose Estimator").setValue(_poseEstimator.getEstimatedPosition().toString());

  }

  public void flipGyro() {
    resetGyroAngle(getGyroAngle() + 180);
    setDesiredHeading(getGyroAngle());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return _sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return _sysIdRoutine.dynamic(direction);
  }
}