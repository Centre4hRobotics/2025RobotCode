package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants;

public class Scorer extends SubsystemBase {

    private SparkFlex _rotationMotor;
    private SparkClosedLoopController _rotationPID;
    private final RelativeEncoder _rotationEncoder;


    private SparkFlex _scoringMotor;
    private SparkClosedLoopController _scoringPID;

    private double _setpoint;

    public Scorer() {
        _rotationMotor = new SparkFlex(31, MotorType.kBrushless);
        _scoringMotor = new SparkFlex(32, MotorType.kBrushless);

        configRotationMotor();
        configScoringMotor();

        _rotationPID = _rotationMotor.getClosedLoopController();
        _scoringPID = _scoringMotor.getClosedLoopController();

        _rotationEncoder = _rotationMotor.getEncoder();

        _setpoint = 0;
    }

    public void setScoringVelocity(double velocity) {
        _scoringPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setRotation(double position) {
        _setpoint = position;
        _rotationPID.setReference(position, ControlType.kPosition);
    }

    public void setRotationVoltage(double voltage) {
        _rotationMotor.setVoltage(voltage);
    }

    public void setScoringVoltage(double voltage) {
        _scoringMotor.setVoltage(voltage);
    }

    public boolean safeToElevateCoral() {
        return _rotationEncoder.getPosition() > ScorerConstants.lowestElevatingRotationCoral 
        && _rotationEncoder.getPosition() < ScorerConstants.highestElevatingRotationCoral;
    }

    public boolean safeToElevateAlgae() {
        return _rotationEncoder.getPosition() > ScorerConstants.lowestElevatingRotationAlgae 
        && _rotationEncoder.getPosition() < ScorerConstants.highestElevatingRotationAlgae;
    }

    // rotation of scorer is clear of crossbeam
    public boolean safeToElevate() {
        return safeToElevateAlgae() || safeToElevateCoral();
    }

    public double getRotation() {
        return _rotationEncoder.getPosition();
    }

    public double getSetpoint()
    {
        return _setpoint;
    }

    public boolean isOnTarget(double target) {
        return Math.abs(getRotation() - target) < ScorerConstants.rotationTolerance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("Scorer").getEntry("rotation encoder value").setValue(getRotation());
    }

    private void configScoringMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(ScorerConstants.scoringCurrentThreshold);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        _scoringMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configRotationMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(ScorerConstants.rotationCurrentThreshold);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ScorerConstants.rotationP, ScorerConstants.rotationI, ScorerConstants.rotationD);
        _rotationMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
