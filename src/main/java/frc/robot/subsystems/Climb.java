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
    private final RelativeEncoder _climbEncoder;


    public Climb() {
        _climber = new SparkMax(50, MotorType.kBrushless);

        configureClimber();

        _climbEncoder = _climber.getEncoder();
        _climbEncoder.setPosition(0.0);
    }

    public void setVoltage(double voltage) {
        _climber.setVoltage(voltage);
    }

    public void syncEncoder() {
        _climbEncoder.setPosition(0);
    }

    public double getPosition() {
        return _climbEncoder.getPosition();
    }

    public double getCurrent() {
        return _climber.getOutputCurrent();
    }

    private void configureClimber() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.smartCurrentLimit(ClimbConstants.climbingCurrentThreshold);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        _climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
