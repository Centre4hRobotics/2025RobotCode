package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FunnelConstants;

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

public class Funnel extends SubsystemBase {

    private SparkMax _funnelMotor;
    private SparkClosedLoopController _funnelPID;
    private final RelativeEncoder _funnelEncoder;

    public Funnel() {
        _funnelMotor = new SparkMax(51, MotorType.kBrushless);
        configFunnelMotor();
        _funnelPID = _funnelMotor.getClosedLoopController();
        _funnelEncoder = _funnelMotor.getEncoder();
    }

    public void setHeight(double position) {
        _funnelPID.setReference(position, ControlType.kPosition);
    }

    public double getHeight() {
        return _funnelEncoder.getPosition();
    }

    private void configFunnelMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);

        config.encoder.positionConversionFactor(1000);
        config.encoder.velocityConversionFactor(1000);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(FunnelConstants.funnelP, FunnelConstants.funnelI, FunnelConstants.funnelD);
        _funnelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
