package frc.robot.subsystems.arm;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.kArmPivot;

public class ArmPivotIOTalonFX implements ArmPivotIO {
    private TalonFX armMotor, followerArmMotor;
    private final PositionVoltage positionVoltage;
    public static double conversionFactor = 0;
    private final double motorArmRatio = 0;

    public ArmPivotIOTalonFX(int ArmPivotIOTalonFX) {

        armMotor = new TalonFX(0);
        followerArmMotor = new TalonFX(0);

        // followerArmMotor.set(ControlModeValue.Follower, armMotor);

        var configurator = armMotor.getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = 30;
        limitConfigs.StatorCurrentLimitEnable = true;
        configurator.apply(limitConfigs);

        var feedBackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(1);
        configurator.apply(feedBackConfig);

        armMotor.setNeutralMode(NeutralModeValue.Brake);

        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        positionVoltage = new PositionVoltage(0).withSlot(0);

        //armMotor.setInverted(false);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kArmPivot.kP;
        slot0Configs.kI = kArmPivot.kI;
        slot0Configs.kD = kArmPivot.kD;

        armMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }

    @Override
    public void setPositionUp(double position) {
        armMotor.setControl(positionVoltage.withPosition(10));
    }

    @Override
    public void setPositionDown(double position) {
        armMotor.setControl(positionVoltage.withPosition(-10));
    }

    @Override
    public void keepPosition(double armPositionRad) {
        var motorPositionRad = armPositionRad*motorArmRatio;
        armMotor.setControl(positionVoltage.withPosition(motorPositionRad));
    }

    @Override
    public void updateInputs(ArmPivotInputs inputs) {
        inputs.armConnected = armMotor.getFaultField().getValue() == 0;
        inputs.armVolts = armMotor.getMotorVoltage().getValueAsDouble();
        inputs.armCurrent = armMotor.getStatorCurrent().getValueAsDouble();
        inputs.armPosition = armMotor.getPosition().getValueAsDouble();
        inputs.armTemp = armMotor.getDeviceTemp().getValueAsDouble();
        inputs.armSpeed = armMotor.getVelocity().getValueAsDouble();
        inputs.armPositionRad = Units.rotationsToRadians(inputs.armPosition);
    }
}