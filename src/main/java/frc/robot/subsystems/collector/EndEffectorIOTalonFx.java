package frc.robot.subsystems.collector;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class EndEffectorIOTalonFx implements EndEffectorIO {
    
    private TalonFX endEffectorMotor;
    private TalonFXConfigurator endEffectorConfig;
    private CurrentLimitsConfigs currentConfig;
    // private Slot0Configs PID;
    // private PositionVoltage m_request;
    // private double kP, kI, kD;

    // private final ShuffleboardTab sb_EndEffector;
    // private final GenericEntry sb_kP;
    // private final GenericEntry sb_kI;
    // private final GenericEntry sb_kD;

    public EndEffectorIOTalonFx(int ID) {
        endEffectorMotor = new TalonFX(ID);
        endEffectorConfig = endEffectorMotor.getConfigurator();
        currentConfig = new CurrentLimitsConfigs();

        currentConfig.SupplyCurrentLimit = 30.0;
        currentConfig.SupplyCurrentLimitEnable = true;
        endEffectorConfig.apply(currentConfig);

        endEffectorMotor.setNeutralMode(NeutralModeValue.Brake);
        // PID.kP = 0.0;
        // PID.kI = 0.0;
        // PID.kD = 0.0;
        // endEffectorConfig.apply(PID);

        // sb_EndEffector = Shuffleboard.getTab("End Effector-TalonFX");
        // sb_kP = sb_EndEffector.add("kP", kP).getEntry();
        // sb_kI = sb_EndEffector.add("kI", kI).getEntry();
        // sb_kD = sb_EndEffector.add("kD", kD).getEntry();

    }

    @Override 
    public void setVoltage(double volts) {
        endEffectorMotor.setVoltage(volts);
    }

    @Override 
    public void updateInputs(EndEffectorInput inputs) {
        
    }

    public void debugPID() {

    }

    public double rotationsToMeters(double motorRotations) {
        double drumRadius = 0.0;
        double circumfrence = 2 * Math.PI * drumRadius;
        double distanceMeters = motorRotations * circumfrence;
        return distanceMeters;
    }
}
