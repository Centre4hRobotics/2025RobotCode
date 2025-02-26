package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ScorerConstants;

public class Climb extends SubsystemBase{
    private SparkMax _climber;
    private SparkClosedLoopController _climbPID;
    private final RelativeEncoder _climbEncoder;

    private double _setpoint;

    public Climb() {
        _climber = new SparkMax(50, MotorType.kBrushless);

        configureClimber();

        _climbPID = _climber.getClosedLoopController();

        _climbEncoder = _climber.getEncoder();
        _climbEncoder.setPosition(0.0);

        _setpoint = 0.0;
    }

    public void setScoringVelocity(double velocity) {
        _climbPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setRotation(double position) {
        _setpoint = position;
        _climbPID.setReference(position, ControlType.kPosition);
    }

    public void setRotationVoltage(double voltage) {
        _climber.setVoltage(voltage);
    }

    public void syncRotationEncoder() {
        _climbEncoder.setPosition(0);
    }

    public double getRotation() {
        return _climbEncoder.getPosition();
    }

    public double getScoringCurrent() {
        return _climber.getOutputCurrent();
    }

    public double getScoringVelocity()
    {
        return _climber.getEncoder().getVelocity();
    }

    public double getScoringPosition() {
        return _climbEncoder.getPosition();
    }

    public double getSetpoint()
    {
        return _setpoint;
    }

    public boolean isOnTarget(double target) {
        return Math.abs(getRotation() - target) < ClimbConstants.climbRotationTolerance;
    }

    private void configureClimber() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(ClimbConstants.climbingCurrentThreshold);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ClimbConstants.climbingP, ClimbConstants.climbingI, ClimbConstants.climbingD);

        _climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
