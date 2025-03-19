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

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

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

        _setpoint = 0;
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

    public void setPosition(double position) {
        _climbPID.setReference(position, ControlType.kPosition);
        _setpoint = position;
    }

    public double getSetpoint()
    {
        return _setpoint;
    }

    public boolean isOnTarget(double target) {
        return Math.abs(getPosition() - target) < ClimbConstants.rotationTolerance;
    }

    public double getCurrent() {
        return _climber.getOutputCurrent();
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("Climb").getEntry("climb encoder value").setValue(getPosition());
        nt.getTable("Climb").getEntry("climb setpoint").setValue(getSetpoint());
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
