package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Elevator extends SubsystemBase {

    private SparkMax _elevatorMotor;
    private SparkClosedLoopController _elevatorPID;
    private final RelativeEncoder _elevatorEncoder;

    public Elevator() {
        _elevatorMotor = new SparkMax(21, MotorType.kBrushless);
        configElevatorMotor();
        _elevatorPID = _elevatorMotor.getClosedLoopController();
        _elevatorEncoder = _elevatorMotor.getEncoder();
    }

    public void setHeight(double position) {
        _elevatorPID.setReference(position, ControlType.kPosition);
    }

    public double getHeight() {
        return _elevatorEncoder.getPosition();
    }

    private void configElevatorMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(1000);
        config.encoder.velocityConversionFactor(1000);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(ElevatorConstants.elevatorP, ElevatorConstants.elevatorI, ElevatorConstants.elevatorD);
        _elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
