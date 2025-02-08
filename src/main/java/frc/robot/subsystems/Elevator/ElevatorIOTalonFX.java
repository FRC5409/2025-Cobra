package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.kElevator;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX m_mainMotor;
    private final TalonFX m_followerMotor;

    private TalonFXConfigurator m_mainMotorConfig;
    private TalonFXConfigurator m_followerMotorConfig;

    private CurrentLimitsConfigs m_currentConfig;

    private FeedbackConfigs m_encoderConfigs;
    private Slot0Configs m_pidConfig = new Slot0Configs();

    private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    private StatusSignal<Voltage> mainDeviceVoltage;
    private StatusSignal<Current> mainDeviceCurrent;
    private StatusSignal<Temperature> mainDeviceTemp;
    private StatusSignal<Voltage> secondaryrDeviceVoltage;
    private StatusSignal<Current> secondaryDeviceCurrent;
    private StatusSignal<Temperature> secondaryDeviceTemp;

    public ElevatorIOTalonFX(int mainMotorID, int followerMotorID) {
        m_mainMotor = new TalonFX(mainMotorID);
        m_followerMotor = new TalonFX(followerMotorID);

        m_mainMotorConfig = m_mainMotor.getConfigurator();
        m_followerMotorConfig = m_followerMotor.getConfigurator();

        m_currentConfig = new CurrentLimitsConfigs();
        m_currentConfig.SupplyCurrentLimit = kElevator.CURRENT_LIMIT;
        m_currentConfig.SupplyCurrentLimitEnable = true;
        
        m_mainMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followerMotor.setNeutralMode(NeutralModeValue.Brake);

        m_encoderConfigs = new FeedbackConfigs();
        m_encoderConfigs.SensorToMechanismRatio = kElevator.kRotationConverter;

        //PID
        m_pidConfig.kP = kElevator.kP; 
        m_pidConfig.kI = kElevator.kI;
        m_pidConfig.kD = kElevator.kD;

        m_mainMotorConfig.apply(m_currentConfig);
        m_followerMotorConfig.apply(m_currentConfig);

        m_mainMotorConfig.apply(m_pidConfig);
        m_followerMotorConfig.apply(m_pidConfig);

        mainDeviceVoltage = m_mainMotor.getMotorVoltage();
        mainDeviceCurrent = m_mainMotor.getSupplyCurrent();
        mainDeviceTemp  = m_mainMotor.getDeviceTemp();
        secondaryrDeviceVoltage = m_followerMotor.getMotorVoltage();
        secondaryDeviceCurrent = m_followerMotor.getSupplyCurrent();
        secondaryDeviceTemp = m_followerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            mainDeviceVoltage,
            mainDeviceCurrent,
            mainDeviceTemp,
            secondaryrDeviceVoltage,
            secondaryDeviceCurrent, 
            secondaryDeviceTemp
        );
        m_mainMotor.optimizeBusUtilization();
        m_followerMotor.optimizeBusUtilization();
    }

    @Override
    public void setMotorVoltage(double volts) {
        m_mainMotor.setVoltage(volts);
    }

    @Override
    public void stopMotor() {
        m_mainMotor.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        m_mainMotor.setPosition(0);
    }

    @Override
    public Distance getPosition() {
        return Meters.of(m_mainMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setSetpoint(Distance setpoint) {
        m_mainMotor.setControl(m_request.withPosition(setpoint.in(Meters)));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.mainMotorConnection = BaseStatusSignal.refreshAll(
            mainDeviceVoltage, 
            mainDeviceCurrent, 
            mainDeviceTemp
        ).isOK();
        inputs.mainAppliedVoltage = mainDeviceVoltage.getValueAsDouble();
        inputs.mainAppliedCurrent = mainDeviceCurrent.getValueAsDouble();
        inputs.mainMotorTemperature = mainDeviceTemp.getValueAsDouble();
        inputs.mainMotorPosition = m_mainMotor.getPosition().getValueAsDouble()*kElevator.kRotationConverter;
        
        inputs.followerMotorConnection = BaseStatusSignal.refreshAll(
            secondaryrDeviceVoltage, 
            secondaryDeviceCurrent, 
            secondaryDeviceTemp
            ).isOK();
            inputs.followerAppliedVoltage = secondaryrDeviceVoltage.getValueAsDouble();
            inputs.followerAppliedCurrent = secondaryDeviceCurrent.getValueAsDouble();
            inputs.followerMotorTemperature = secondaryDeviceTemp.getValueAsDouble();        
            inputs.followerMotorPosition = m_followerMotor.getPosition().getValueAsDouble()*kElevator.kRotationConverter;
    }
}
