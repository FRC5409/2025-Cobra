package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.kArmPivot;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private TalonFX armMotor;
    private CANcoder canCoderSensor;
    private final PositionVoltage positionVoltage;
    private final double motorArmRatio = 1;
    private final double sensorOffset = 0.0;

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

        positionVoltage = new PositionVoltage(0).withSlot(0);

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = kArmPivot.kP;
        slot0Configs.kI = kArmPivot.kI;
        slot0Configs.kD = kArmPivot.kD;
        slot0Configs.kG = kArmPivot.kG;

        armMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }
    
    @Override
    public void moveArm(Angle armPositionRad) {
        armMotor.setControl(positionVoltage.withPosition(armPositionRad));
    }

    @Override
    public void updateInputs(ArmPivotInputs inputs) {
        inputs.connected = armMotor.getFaultField().getValue() == 0;
        inputs.voltage = armMotor.getMotorVoltage().getValueAsDouble();
        inputs.current = armMotor.getSupplyCurrent().getValueAsDouble();
        inputs.positionAngles = armMotor.getPosition().getValue().in(Degrees);
        inputs.temperature = armMotor.getDeviceTemp().getValue().in(Celsius);
        inputs.speed = armMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.positionRad = Units.rotationsToRadians(inputs.positionAngles);
    }
}