package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Kelvin;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.kElevator;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX m_mainMotor;
    private final TalonFX m_followerMotor;

    private TalonFXConfigurator m_mainMotorConfig;
    private TalonFXConfigurator m_followerMotorConfig;

    private CurrentLimitsConfigs m_currentConfig;

    private FeedbackConfigs m_encoderConfigs;
    private Slot0Configs m_pidConfig = new Slot0Configs();

    private PositionVoltage m_request = new PositionVoltage(0).withSlot(0);;
    private ShuffleboardTab sb_elevator;    

    public ElevatorIOTalonFX(int mainMotorID, int followerMotorID) {
        m_mainMotor = new TalonFX(mainMotorID);
        m_followerMotor = new TalonFX(followerMotorID);

        m_mainMotorConfig = m_mainMotor.getConfigurator();
        m_followerMotorConfig = m_followerMotor.getConfigurator();

        m_currentConfig = new CurrentLimitsConfigs();
        m_currentConfig.SupplyCurrentLimit = kElevator.CURRENT_LIMIT;
        m_currentConfig.SupplyCurrentLimitEnable = kElevator.CURRENT_CONFIG;
        
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
        
        sb_elevator  = Shuffleboard.getTab("Elevator");
        sb_elevator.addDouble("Position", ()->m_mainMotor.getPosition().getValueAsDouble());
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
    public double getPosition() {
        return m_mainMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setSetpoint(double setpoint) {
        m_mainMotor.setControl(m_request.withPosition(setpoint));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.mainMotorConnected = m_mainMotor.isAlive();
        inputs.mainAppliedVoltage = m_mainMotor.get()*RobotController.getBatteryVoltage();
        inputs.mainAppliedCurrent = m_mainMotor.getSupplyCurrent().getValueAsDouble();
        inputs.mainMotorTemperature = m_mainMotor.getDeviceTemp().getValueAsDouble();
        inputs.mainMotorPosition = m_mainMotor.getPosition().getValueAsDouble()*kElevator.kRotationConverter;
        
        inputs.followerMotorConnected = m_followerMotor.isAlive();
        inputs.followerAppliedVoltage = m_followerMotor.get()*RobotController.getBatteryVoltage();
        inputs.followerAppliedCurrent = m_followerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.followerMotorTemperature = m_followerMotor.getDeviceTemp().getValueAsDouble();
        inputs.followerMotorPosition = m_followerMotor.getPosition().getValueAsDouble()*kElevator.kRotationConverter;
    }
}
