package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.kArmPivot;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private TalonFX armMotor;
    private CANcoder canCoderSensor;
    private final PositionTorqueCurrentFOC positionVoltage;
    private final double motorArmRatio = 1;
    private final double sensorOffset = 0.0;

    private StatusSignal<Angle> positionSignal;
    private StatusSignal<AngularVelocity> velocitySignal;
    private StatusSignal<Voltage> deviceVoltage;
    private StatusSignal<Current> deviceCurrent;
    private StatusSignal<Temperature> deviceTemp;

    public ArmPivotIOTalonFX(int canID, int sensorID) {

        armMotor = new TalonFX(canID);
        canCoderSensor = new CANcoder(sensorID);

        TalonFXConfigurator configurator = armMotor.getConfigurator();
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.SupplyCurrentLimit = 30;
        limitConfigs.SupplyCurrentLimitEnable = true;
        configurator.apply(limitConfigs);

        FeedbackConfigs feedBackConfig = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRemoteCANcoder(canCoderSensor)
            .withSensorToMechanismRatio(motorArmRatio)
            .withFeedbackRotorOffset(sensorOffset);
            
        configurator.apply(feedBackConfig);

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        positionVoltage = new PositionTorqueCurrentFOC(0).withSlot(0);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kArmPivot.kP;
        slot0Configs.kI = kArmPivot.kI;
        slot0Configs.kD = kArmPivot.kD;
        slot0Configs.kG = kArmPivot.kG;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        armMotor.getConfigurator().apply(slot0Configs);

        positionSignal = armMotor.getPosition();
        velocitySignal = armMotor.getVelocity();
        deviceVoltage = armMotor.getMotorVoltage();
        deviceCurrent = armMotor.getSupplyCurrent();
        deviceTemp = armMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            positionSignal,
            velocitySignal,
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        );

        armMotor.optimizeBusUtilization();
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }
    
    @Override
    public void setSetpoint(Angle armPositionRad) {
        armMotor.setControl(positionVoltage.withPosition(armPositionRad));
    }

    @Override
    public Angle getPosition() {
        return armMotor.getPosition().getValue();
    }

    @Override
    public void updateInputs(ArmPivotInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        ).isOK();

        inputs.positionAngles = positionSignal.getValue().in(Degrees);
        inputs.speed = velocitySignal.getValue().in(RadiansPerSecond);
        inputs.positionRad = Units.rotationsToRadians(inputs.positionAngles);
        inputs.voltage = deviceVoltage.getValueAsDouble();
        inputs.current = deviceCurrent.getValueAsDouble();
        inputs.temperature = deviceTemp.getValueAsDouble();
    }
}