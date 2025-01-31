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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScorerConstants;

public class Scorer extends SubsystemBase {

    private SparkFlex _rotationMotor;
    private SparkClosedLoopController _rotationPID;
    private final RelativeEncoder _rotationEncoder;


    private SparkFlex _scoringMotor;
    private SparkClosedLoopController _scoringPID;

    public Scorer() {
        _rotationMotor = new SparkFlex(31, MotorType.kBrushless);
        _scoringMotor = new SparkFlex(32, MotorType.kBrushless);

        configRotationMotor();
        configScoringMotor();

        _rotationPID = _rotationMotor.getClosedLoopController();
        _scoringPID = _scoringMotor.getClosedLoopController();

        _rotationEncoder = _rotationMotor.getEncoder();
    }

    public void setScoringVelocity(double velocity) {
        _scoringPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setRotation(double position) {
        _rotationPID.setReference(position, ControlType.kPosition);
    }

    public void setRotationVoltage(double voltage) {
        _rotationMotor.setVoltage(voltage);
    }

    public void setScoringVoltage(double voltage) {
        _scoringMotor.setVoltage(voltage);
    }

    public double getRotation() {
        return _rotationEncoder.getPosition();
    }

    private void configScoringMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(ScorerConstants.scoringCurrentThreshold);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ScorerConstants.scoringP, ScorerConstants.scoringI, ScorerConstants.scoringD);

        _scoringMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configRotationMotor() {
        SparkFlexConfig config = new SparkFlexConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(1000);
        config.encoder.velocityConversionFactor(1000);

        config.smartCurrentLimit(ScorerConstants.rotationCurrentThreshold);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ScorerConstants.rotationP, ScorerConstants.rotationI, ScorerConstants.rotationD);
        _rotationMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
