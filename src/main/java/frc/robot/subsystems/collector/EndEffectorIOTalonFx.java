package frc.robot.subsystems.collector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.kEndEffector;

public class EndEffectorIOTalonFx implements EndEffectorIO {
    
    private TalonFX endEffectorMotor;
    private TalonFXConfigurator endEffectorConfig;
    private CurrentLimitsConfigs currentConfig;

    private StatusSignal<Voltage> deviceVoltage;
    private StatusSignal<Current> deviceCurrent;
    private StatusSignal<Temperature> deviceTemp;

    public EndEffectorIOTalonFx(int ID) {
        // Creating Objects
        endEffectorMotor = new TalonFX(ID);
        endEffectorConfig = endEffectorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();

        // Setting Configs
        currentConfig.SupplyCurrentLimit = kEndEffector.CURRENT_LIMIT;
        currentConfig.SupplyCurrentLimitEnable = true;
        endEffectorConfig.apply(currentConfig);

        endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);

        // Getting Status Signals
        deviceVoltage = endEffectorMotor.getMotorVoltage();
        deviceCurrent = endEffectorMotor.getSupplyCurrent();
        deviceTemp = endEffectorMotor.getDeviceTemp();
        // Make it so these status signals aren't touched by optimization
        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        );
        // Optimize the CanBus
        endEffectorMotor.optimizeBusUtilization();
    }

    @Override 
    public void setVoltage(double volts) {
        endEffectorMotor.setVoltage(volts);
    }

    @Override 
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.endEffectorConnection = BaseStatusSignal.refreshAll(
            deviceVoltage,
            deviceCurrent,
            deviceTemp).isOK();
        inputs.endEffectorVolts = deviceVoltage.getValueAsDouble();
        inputs.endEffectorCurrent = deviceCurrent.getValueAsDouble();
        inputs.endEffectTemp = deviceTemp.getValueAsDouble();
    }
}
