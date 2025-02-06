package frc.robot.subsystems.collector;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.kEndEffector;
public class EndEffectorIOTalonFx implements EndEffectorIO {
    
    private TalonFX endEffectorMotor;
    private TalonFXConfigurator endEffectorConfig;
    private CurrentLimitsConfigs currentConfig;

    public EndEffectorIOTalonFx(int ID) {
        endEffectorMotor = new TalonFX(ID);
        endEffectorConfig = endEffectorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();

        currentConfig.SupplyCurrentLimit = kEndEffector.CURRENT_LIMIT;
        currentConfig.SupplyCurrentLimitEnable = kEndEffector.CURRENT_CONFIG;
        endEffectorConfig.apply(currentConfig);

        endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override 
    public void setVoltage(double volts) {
        endEffectorMotor.setVoltage(volts);
    }

    @Override 
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.EndEffectorConnection = endEffectorMotor.isAlive();
        inputs.EndEffectorVolts = endEffectorMotor.get()*RobotController.getBatteryVoltage();
        inputs.EndEffectorCurrent = endEffectorMotor.getSupplyCurrent().getValueAsDouble();
        inputs.EndEffectTemp = endEffectorMotor.getDeviceTemp().getValueAsDouble();
    }
}
