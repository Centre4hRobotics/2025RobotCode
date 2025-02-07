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

    private final PositionVoltage _positionVoltage;


    public Elevator() {
        _leadMotor = new TalonFX(21);
        _followMotor = new TalonFX(22);

        configLeadMotor();
        configFollowMotor();

        _positionVoltage = new PositionVoltage(0).withSlot(0);
    }

    public void setHeight(double position) {
        System.out.print(_positionVoltage.withPosition(position));
        _leadMotor.setControl(_positionVoltage.withPosition(position));
    }

    public void setVoltage(double voltage) {
        _leadMotor.set(voltage);
    }

    public double getHeight() {
        return _leadMotor.getPosition().getValueAsDouble();
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

    public boolean isOnTarget(double target) {
        return Math.abs(getHeight() - target) < ElevatorConstants.heightTolerance;
    }

    public double heightFraction() {
        double fraction = getHeight()/ElevatorConstants.maxHeight;
        if(fraction > 0 && fraction <= 1) {return fraction; }
        else if(fraction <= 0) {return 0; }
        else {return 1; }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        nt.getTable("Elevator").getEntry("encoder value").setValue(getHeight());
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

        _leadConfiguration.Voltage.withPeakForwardVoltage(6).withPeakReverseVoltage(-3);

        _leadMotor.getConfigurator().apply(_leadConfiguration);
    }

    private void configFollowMotor() {
        _followMotor.setControl(new Follower(_leadMotor.getDeviceID(), true));
    }
}
