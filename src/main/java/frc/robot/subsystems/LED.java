package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kLED;
import frc.robot.subsystems.collector.EndEffector;
import frc.robot.subsystems.vision.Vision;


public class LED extends SubsystemBase {
  private final CANdle candle;
  private final CANdleConfiguration config;
  private final LEDConfigs ledConfigs;
  
  // Current Color: Yellow (from 5409 color code) (doesn't actually show up as the yellow which was expected)
  private final int[] chargersColor = {127, 87, 0};

  // private final LoggedNetworkNumber brightness = new LoggedNetworkNumber("LED Brightness", 0.5);

  public static enum LEDModes {
    OFF,
    SOLID_YELLOW,
    SOLID_GREEN,
    SOLID_RED,
    SOLID_BLUE,
    BLINKING_YELLOW,
    RAINBOW,
    TWINKLE,
    COLORFLOW_YELLOW,
    RGB_FADE,
    SINGLE_FADE,
    STROBE
  }

  private LEDModes LEDMode;
  private LEDModes previousLEDMode;


  public LED() {

    candle = new CANdle(kLED.LEDCanId);
    config = new CANdleConfiguration();

    LEDMode = LEDModes.OFF;
    previousLEDMode = LEDModes.OFF;

    ledConfigs = new LEDConfigs();
    ledConfigs.BrightnessScalar = 0.5;

    config.withLED(ledConfigs);

    SmartDashboard.putData("LED/LED OFF", this.setLEDModeCommand(LEDModes.OFF).ignoringDisable(true));
    SmartDashboard.putData("LED/LED SOLID YELLOW", this.setLEDModeCommand(LEDModes.SOLID_YELLOW).ignoringDisable(true));
    SmartDashboard.putData("LED/LED SOLID GREEN", this.setLEDModeCommand(LEDModes.SOLID_GREEN).ignoringDisable(true));
    SmartDashboard.putData("LED/LED BLINKING YELLOW", this.setLEDModeCommand(LEDModes.BLINKING_YELLOW).ignoringDisable(true));
    SmartDashboard.putData("LED/LED RAINBOW", this.setLEDModeCommand(LEDModes.RAINBOW).ignoringDisable(true));
    SmartDashboard.putData("LED/LED TWINKLE", this.setLEDModeCommand(LEDModes.TWINKLE).ignoringDisable(true));
    SmartDashboard.putData("LED/LED COLORFLOW_YELLOW", this.setLEDModeCommand(LEDModes.COLORFLOW_YELLOW).ignoringDisable(true));
    SmartDashboard.putData("LED/LED RGB FADE", this.setLEDModeCommand(LEDModes.RGB_FADE).ignoringDisable(true));
    SmartDashboard.putData("LED/LED SINGLE FADE", this.setLEDModeCommand(LEDModes.SINGLE_FADE).ignoringDisable(true));
    SmartDashboard.putData("LED/LED STROBE", this.setLEDModeCommand(LEDModes.STROBE).ignoringDisable(true));

    SmartDashboard.putString("LED/LED MODE", LEDMode.toString());
    // config.LED.withStripType(StripTypeValue.RGB);
    // config.CANdleFeatures.withVBatOutputMode(VBatOutputModeValue.Off);

    candle.getConfigurator().apply(config);

  }

    @Override
    public void periodic() {
        if (LEDMode != previousLEDMode) {
        stopAnimation();
        switch (LEDMode) {
            case OFF:
                setSolid(0, 0, 0, 0);
                break;
            case SOLID_YELLOW:
                setSolid(chargersColor[0], chargersColor[1], chargersColor[2], 0);
                break;
            case SOLID_GREEN:
                setSolid(0, 255, 0, 0);
                break;
            case SOLID_RED:
                setSolid(255, 50, 50, 0);
                break;
            case SOLID_BLUE:
                setSolid(0, 0, 255, 0);
                break;
            case BLINKING_YELLOW:
                setBlinkingSolid(chargersColor[0], chargersColor[1], chargersColor[2], 0, 0.5).schedule();
                break;
            case RAINBOW:
                setRainbowAnimation(0,20);
                break;
            case TWINKLE:
                setTwinkleAnimation(chargersColor[0], chargersColor[1], chargersColor[2], 60, 1);
                break;
            case COLORFLOW_YELLOW:
                setColorFlowAnimation(chargersColor[0], chargersColor[1], chargersColor[2], 60, AnimationDirectionValue.Forward, 2);
                break;
            case RGB_FADE:
                setRGBFadeAnimation(60, 3);
                break;
            case SINGLE_FADE:
                setSingleFadeAnimation(chargersColor[0], chargersColor[1], chargersColor[2], 60, 4);
                break;
            case STROBE:
                setStrobeAnimation(chargersColor[0], chargersColor[1], chargersColor[2], 60, 5);
                break;
            default:
                setSolid(0, 0, 0, 0);
                break;
            
            }
            previousLEDMode = LEDMode;
        };
    }

    /** 
     * Sets LEDs to a solid color. 
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param w White value (0-255)
     * */
    private void setSolid(int r, int g, int b, int w) {
        candle.setControl(
            new SolidColor(0, kLED.numOfLED + 7)
            .withColor(new RGBWColor(r, g, b, w)));
    }
    /** 
     * Command that makes the LEDs blink a solid color.
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param w White value (0-255)
     * @param interval Time in seconds for one on/off cycle
     * */
    private Command setBlinkingSolid(int r, int g, int b, int w, double interval) {
        return Commands.sequence(
            // Commands.runOnce(() -> this.setSolid(r, g, b, w)),
            // Commands.waitSeconds(interval),
            // Commands.runOnce(() -> this.setSolid(0, 0, 0, 0)),
            // Commands.waitSeconds(interval),
            Commands.runOnce(() -> this.setSolid(r, g, b, w)),
            Commands.waitSeconds(interval),
            Commands.runOnce(() -> this.setSolid(0, 0, 0, 0))
            // Commands.waitSeconds(interval),
            // Commands.runOnce(() -> this.setSolid(r, g, b, w)),
            // Commands.waitSeconds(interval),
            // Commands.runOnce(() -> this.setSolid(0, 0, 0, 0))
        ).ignoringDisable(true);
    }

    /** 
     * Sets a rainbow animation on the LED strip.
     * @param slot Animation slot to use
     * @param frameRate Frame rate of the animation
     * */
    private void setRainbowAnimation(int slot, int frameRate) {
        candle.setControl(
            new RainbowAnimation(0, kLED.numOfLED + 7)
                .withDirection(AnimationDirectionValue.Forward)
                .withFrameRate(frameRate)
                .withSlot(slot)
            );        
    }

    /** 
     * Stops any running animation on all slots.
     * */
    private void stopAnimation() {
        for (int i = 0; i < 8; i++) {
        candle.setControl(new EmptyAnimation(i));
        }
    }
  
    /**
     * Animation that randomly turn leds on and off to a certain color.
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param frameRate Frame rate of the animation
     * @param slot
     */
    private void setTwinkleAnimation(int r, int g, int b, int frameRate, int slot) {
        candle.setControl(
            new TwinkleAnimation(0, kLED.numOfLED + 7)
                .withColor(new RGBWColor(r, g, b))
                .withFrameRate(frameRate)
                .withSlot(slot)
        );
    }

    /**
     * Animation that gradually lights the entire LED strip one LED at a time.
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param frameRate Frame rate of the animation
     * @param direction Animation flow direction
     * @param slot Animation slot to use
     */
    private void setColorFlowAnimation(int r, int g, int b, int frameRate, AnimationDirectionValue direction ,int slot) {
        candle.setControl(
            new ColorFlowAnimation(0,  kLED.numOfLED + 7)
                .withColor(new RGBWColor(r, g, b))
                .withFrameRate(frameRate)
                .withDirection(direction)
                .withSlot(slot)
        );
    }

    /**
     * Animation that fades all the LEDs of a strip simultaneously between Red, Green, and Blue
     * @param frameRate Frame rate of the animation
     * @param slot Animation slot to use
     */
    private void setRGBFadeAnimation (int frameRate, int slot){
        candle.setControl(
        new RgbFadeAnimation(0, kLED.numOfLED + 7)
            .withFrameRate(frameRate)
            .withSlot(slot)
        );
    }

    /**
     * Animation that fades into and out of a specified color
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param frameRate Frame rate of the animation
     * @param slot Animation slot to use
     */
    private void setSingleFadeAnimation (int r, int g, int b, int frameRate, int slot){
        candle.setControl(
        new SingleFadeAnimation(0, kLED.numOfLED + 7)
            .withColor(new RGBWColor(r, g, b))
            .withFrameRate(frameRate)
            .withSlot(slot)
        );
    }

    /**
     * Animation that strobes the LEDs a specified color
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param frameRate Frame rate of the animation
     * @param slot Animation slot to use
     */
    private void setStrobeAnimation(int r, int g, int b, int frameRate, int slot) {
        candle.setControl(
        new StrobeAnimation(0, kLED.numOfLED + 7)
            .withColor(new RGBWColor(r, g, b))
            .withFrameRate(frameRate)
            .withSlot(slot)
        );
    }

    /**
     * Sets the current LED mode
     * @param inputLEDMode The LED mode to set
     */
    private void setLEDMode(LEDModes inputLEDMode) {
        LEDMode = inputLEDMode;
    }

    /**
     * @return The current LED mode
     */
    private LEDModes getLEDMode() {
        return LEDMode;
    }

    /**
     * Command to set the LED mode
     * @param inputLEDMode
     * @return Command that sets the LED mode
     */
    public Command setLEDModeCommand(LEDModes inputLEDMode) {
        return Commands.runOnce(() -> this.setLEDMode(inputLEDMode)).ignoringDisable(true);
    }
    
    /**
     * @return the current LED mode
     */
    public Command getLEDModeCommand() {
        return Commands.runOnce(() -> this.getLEDMode());
    }

    /**
     * Default LED command that sets the LED mode based on vision and end effector status.
     * If coral is detected, blinks yellow first, then solid green if target is detected, solid yellow if not.
     * If coral is not detected, if it is connected to fms and aahan controls are set, sets it to rainbow mode
     * otherwise solid blue. If not connected to fms, turns off the LEDs (for pit crew).
     * @param vision Vision subsystem
     * @param endEffector End effector subsystem
     * @param isConnected Whether the driverstation is connected the FMS
     * @param aahanControls Supplier for whether Aahan controls is true
     * @return Command that sets the LED mode based on vision and end effector status
     */
    public Command setLEDDefault(Vision vision, EndEffector endEffector, boolean isConnected, BooleanSupplier aahanControls){
        return Commands.run(
            () -> {
                if (endEffector.coralDetected()) {
                    if (vision.hasTarget()) {
                        setLEDMode(LEDModes.SOLID_GREEN);
                    } else {
                        setLEDMode(LEDModes.SOLID_YELLOW);
                    }
                } else if (isConnected){
                    if (aahanControls.getAsBoolean()) {
                        setLEDMode(LEDModes.RAINBOW);
                    } else {
                        setLEDMode(LEDModes.SOLID_BLUE);
                    }
                } else {
                    setLEDMode(LEDModes.OFF);
                }
            },
            this, endEffector, vision
        ).ignoringDisable(true);
    }
}