package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;
//୨୧

public class KrakenSwerveModule extends SwerveModuleBase {

    private TalonFX _drivingMotor;
    private TalonFX _turningMotor;

    // what is applied to the motors
    private TalonFXConfiguration _drivingConfiguration;
    private TalonFXConfiguration _turningConfiguration;

    // stores configuration for the motor (technically not needed :))
    private TalonFXConfigurator _drivingConfigurator;
    private TalonFXConfigurator _turningConfigurator;

    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage anglePosition = new PositionVoltage(0);

  /** Creates a new SwerveModule. */
  public KrakenSwerveModule (int driveDeviceId, int turnDeviceId, int absoluteEncoderId, double absoluteEncoderAngleOffset, String name) {
    super(absoluteEncoderId, absoluteEncoderAngleOffset, name);

    _drivingMotor = new TalonFX(driveDeviceId);
    _turningMotor = new TalonFX(turnDeviceId);

    _drivingConfiguration = new TalonFXConfiguration();
    _turningConfiguration = new TalonFXConfiguration();

    _drivingConfigurator = _drivingMotor.getConfigurator();
    _turningConfigurator = _turningMotor.getConfigurator();

    configDriveMotor();
    configTurnMotor();

    resetDesiredState();
    
    // syncs encoder
    syncEncoder();
}

  @Override
  public void setPIDReference(SwerveModuleState swerveModuleState) {
    driveVelocity.Velocity = MPSToRPS(swerveModuleState.speedMetersPerSecond, RobotConstants.wheelCircumference);
    _drivingMotor.setControl(driveVelocity); 
    _turningMotor.setControl(anglePosition.withPosition(swerveModuleState.angle.getRotations()));
  }

  @Override
  public void setDriveSpeed(double speed) {
    _drivingMotor.set(speed);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    _drivingMotor.setVoltage(voltage);
  }

  @Override
  public void setRotationSpeed(double speed){
    _turningMotor.set(speed);
  }

  @Override
  public void setRelativeRotationPosition(double position) {
    _turningMotor.setPosition(position/2/Math.PI);
  }

  @Override
  public double getDriveVelocity() {
    return RPSToMPS(_drivingMotor.getVelocity().getValueAsDouble(), RobotConstants.wheelCircumference);
  }

  @Override
  public double getDrivePosition() {
    return rotationsToMeters(_drivingMotor.getPosition().getValueAsDouble(), RobotConstants.wheelCircumference);
  }

  

  @Override
  public double getRelativeRotationPositionRad() {
    return correctAngle(_turningMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI);
  }

  // figure out what config stuff we want yay
  private void configDriveMotor() {
    _drivingConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    _drivingConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    _drivingConfiguration.CurrentLimits.StatorCurrentLimit = MotorConstants.driveCurrentThreshold; 
    _drivingConfiguration.CurrentLimits.SupplyTimeThreshold = MotorConstants.driveTimeThreshold;

    _drivingConfiguration.Feedback.SensorToMechanismRatio = RobotConstants.driveGearRatio;
    
    _drivingConfiguration.Slot0.kP = MotorConstants.drivingP;
    _drivingConfiguration.Slot0.kI = MotorConstants.drivingI;
    _drivingConfiguration.Slot0.kD = MotorConstants.drivingD;
    _drivingConfiguration.Slot0.kV = MotorConstants.drivingFF;

    _drivingConfigurator.apply(_drivingConfiguration);
  }

  // figure out what config stuff we want yay !!
  private void configTurnMotor() {
    _turningConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    _turningConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    _turningConfiguration.CurrentLimits.StatorCurrentLimit = MotorConstants.steerCurrentThreshold; 
    _turningConfiguration.CurrentLimits.SupplyTimeThreshold = MotorConstants.steerTimeThreshold;

    _turningConfiguration.Feedback.SensorToMechanismRatio = RobotConstants.steerGearRatio;
    _turningConfiguration.ClosedLoopGeneral.ContinuousWrap = true;    
    
    _turningConfiguration.Slot0.kP = MotorConstants.turningP;

    _turningConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    _turningConfigurator.apply(_turningConfiguration);
  }

  public void setValues(double... values) {}

  @Override
  public void setTurnEncoderPIDTarget(double target) {
    anglePosition.Position = target;
    _turningMotor.setControl(anglePosition);
  }

  private double RPSToMPS(double wheelRPS, double circumference) {
    double wheelMPS = wheelRPS * (circumference * 2.54 / 100);
    return wheelMPS;
  }

  private double MPSToRPS(double wheelMPS, double circumference) {
    double wheelRPS = wheelMPS / (circumference * 2.54 / 100);
    return wheelRPS;
  }
  
  private double rotationsToMeters(double wheelRotations, double circumference){
    double wheelMeters = wheelRotations * (circumference * 2.54 / 100);
    return wheelMeters;
  }
}