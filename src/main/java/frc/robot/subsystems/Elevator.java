package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.RobotConstants;



public class Elevator extends SubsystemBase {

    private TalonFX _leadMotor;
    private TalonFX _followMotor;

    public Elevator() {
        _leadMotor = new TalonFX(21);
        _followMotor = new TalonFX(22);

        configLeadMotor();
        configFollowMotor();
    }

    public void setHeight(double position) {
        _leadMotor.setPosition(position);
    }

    public void setVoltage(double voltage) {
        _leadMotor.set(voltage);
    }

    public double getHeight() {
        return _leadMotor.get();
    }

    public boolean belowCrossbeam() {
        return _leadMotor.get() < ElevatorConstants.heightBelowCrossbeam;
    }

    public boolean aboveCrossbeam() {
        return _leadMotor.get() > ElevatorConstants.heightAboveCrossbeam;
    }

    public boolean clearCrossbeam() {
        return aboveCrossbeam() || belowCrossbeam();
    }

    private void configLeadMotor() {
        TalonFXConfiguration _leadConfiguration = new TalonFXConfiguration();
        _leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        _leadConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        _leadConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        _leadConfiguration.CurrentLimits.StatorCurrentLimit = ElevatorConstants.leadCurrentThreshold; 

        _leadConfiguration.Slot0.kP = ElevatorConstants.elevatorP;
        _leadConfiguration.Slot0.kI = ElevatorConstants.elevatorI;
        _leadConfiguration.Slot0.kD = ElevatorConstants.elevatorD;

        _leadMotor.getConfigurator().apply(_leadConfiguration);
    }

    private void configFollowMotor() {
        _followMotor.setControl(new Follower(_leadMotor.getDeviceID(), true));
    }
}
