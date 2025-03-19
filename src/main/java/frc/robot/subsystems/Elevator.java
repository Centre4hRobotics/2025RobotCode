package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;



public class Elevator extends SubsystemBase {

    private final TalonFX _leadMotor;
    private final TalonFX _followMotor;

    private double _setpoint;

    private final PositionVoltage _positionVoltage;


    public Elevator() {
        _leadMotor = new TalonFX(21);
        _followMotor = new TalonFX(22);

        _leadMotor.setPosition(0);
        _followMotor.setPosition(0);

        _setpoint = 0;

        configLeadMotor();
        configFollowMotor();

        _positionVoltage = new PositionVoltage(0).withSlot(0);
    }

    public void setHeight(double position) {
        _setpoint = position;
        _leadMotor.setControl(_positionVoltage.withPosition(position));
    }

    public void setVoltage(double voltage) {
        _leadMotor.set(voltage);
    }

    public double getHeight() {
        return _leadMotor.getPosition().getValueAsDouble();
    }

    public double getSetpoint() {
        return _setpoint;
    }

    public boolean isOnTarget(double target) {
        return Math.abs(getHeight() - target) < ElevatorConstants.heightTolerance;
    }

    public double getHeightFraction() {    
        double heightDiff = getHeight() - ElevatorConstants.maxFullSpeedHeight;
        if(heightDiff > 0) {
            return 1 - heightDiff/(ElevatorConstants.maxHeight - ElevatorConstants.maxFullSpeedHeight) * (1-ElevatorConstants.maxHeightVelocityPercent);
        } else {
            return 1;
        }
    }

    public void syncEncoders() {
        _leadMotor.setPosition(0);
        setHeight(0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("Elevator").getEntry("elevator encoder value").setValue(getHeight()); 
        nt.getTable("Elevator").getEntry("elevator setpoint").setValue(_setpoint); 
    }

    private void configLeadMotor() {
        TalonFXConfiguration _leadConfiguration = new TalonFXConfiguration();
        _leadConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        _leadConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        _leadConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        _leadConfiguration.CurrentLimits.StatorCurrentLimit = ElevatorConstants.leadCurrentThreshold; 

        _leadConfiguration.Slot0.kP = ElevatorConstants.elevatorP;
        _leadConfiguration.Slot0.kI = ElevatorConstants.elevatorI;
        _leadConfiguration.Slot0.kD = ElevatorConstants.elevatorD;

        _leadConfiguration.Voltage.withPeakForwardVoltage(9).withPeakReverseVoltage(-6);

        _leadMotor.getConfigurator().apply(_leadConfiguration);
    }

    private void configFollowMotor() {
        _followMotor.setControl(new Follower(_leadMotor.getDeviceID(), true));
    }
}
