package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kLED;


public class LED extends SubsystemBase {
  private final CANdle candle;
  private final CANdleConfiguration config;
  private final LEDConfigs ledConfigs;
  private final Timer timer;
  
  // Current Color: Yellow (from 5409 color code) (doesn't actually show up as the yellow which was expected)
  private final int[] color = {127, 87, 0};

  // private final LoggedNetworkNumber brightness = new LoggedNetworkNumber("LED Brightness", 0.5);

  public static enum LEDModes {
    OFF,
    SOLID,
    RAINBOW,
    TWINKLE,
    COLORFLOW,
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
    SmartDashboard.putData("LED/LED SOLID", this.setLEDModeCommand(LEDModes.SOLID).ignoringDisable(true));
    SmartDashboard.putData("LED/LED RAINBOW", this.setLEDModeCommand(LEDModes.RAINBOW).ignoringDisable(true));
    SmartDashboard.putData("LED/LED TWINKLE", this.setLEDModeCommand(LEDModes.TWINKLE).ignoringDisable(true));
    SmartDashboard.putData("LED/LED COLORFLOW", this.setLEDModeCommand(LEDModes.COLORFLOW).ignoringDisable(true));
    SmartDashboard.putData("LED/LED RGB FADE", this.setLEDModeCommand(LEDModes.RGB_FADE).ignoringDisable(true));
    SmartDashboard.putData("LED/LED SINGLE FADE", this.setLEDModeCommand(LEDModes.SINGLE_FADE).ignoringDisable(true));
    SmartDashboard.putData("LED/LED STROBE", this.setLEDModeCommand(LEDModes.STROBE).ignoringDisable(true));

    SmartDashboard.putString("LED/LED MODE", LEDMode.name());
    // config.LED.withStripType(StripTypeValue.RGB);
    // config.CANdleFeatures.withVBatOutputMode(VBatOutputModeValue.Off);

    candle.getConfigurator().apply(config);

    timer = new Timer();
    timer.start();
  }

  @Override
  public void periodic() {
    if (LEDMode != previousLEDMode) {
      stopAnimation();
      switch (LEDMode) {
        case OFF:
          this.setSolid(0, 0, 0, 0);
          break;
        case SOLID:
            this.setSolid(color[0], color[1], color[2], 0);
            break;
        case RAINBOW:
            this.setRainbowAnimation(0);
            break;
        case TWINKLE:
            this.setTwinkleAnimation(color[0], color[1], color[2], 60, 1);
            break;
        case COLORFLOW:
            this.setColorFlowAnimation(color[0], color[1], color[2], 60, AnimationDirectionValue.Forward, 2);
            break;
        case RGB_FADE:
            this.setRGBFadeAnimation(60, 3);
            break;
        case SINGLE_FADE:
            this.setSingleFadeAnimation(color[0], color[1], color[2], 60, 4);
            break;
        case STROBE:
            this.setStrobeAnimation(color[0], color[1], color[2], 60, 5);
            break;
        default:
            this.setSolid(0, 0, 0, 0);
            break;
        
      }
      previousLEDMode = LEDMode;
    }
    ;
  }

//   Sets LEDs to a solid color.
  public void setSolid(int r, int g, int b, int w) {
    candle.setControl(
        new SolidColor(0, kLED.numOfLED + 7)
            .withColor(new RGBWColor(r, g, b, w)));
  }

//   Animation that creates a rainbow throughout all the LEDs
  public void setRainbowAnimation(int slot) {
    candle.setControl(
        new RainbowAnimation(0, kLED.numOfLED + 7)
            // .withBrightness(0.5)
            .withDirection(AnimationDirectionValue.Forward)
            .withFrameRate(20)
            .withSlot(slot)
        );
            
  }
//   Stops any ongoing animation on the LED strip
  public void stopAnimation() {
    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }
  }
  
//   Animation that randomly turns LEDs on and off to a certain color
  public void setTwinkleAnimation(int r, int g, int b, int frameRate, int slot) {
    candle.setControl(
        new TwinkleAnimation(0, kLED.numOfLED + 7)
            .withColor(new RGBWColor(r, g, b))
            .withFrameRate(frameRate)
            .withSlot(slot)
    );
  }
//   Animation that gradually lights the entire LED strip one LED at a time.
  public void setColorFlowAnimation(int r, int g, int b, int frameRate, AnimationDirectionValue direction ,int slot) {
    candle.setControl(
        new ColorFlowAnimation(0,  kLED.numOfLED + 7)
            .withColor(new RGBWColor(r, g, b))
            .withFrameRate(frameRate)
            .withDirection(direction)
            .withSlot(slot)
    );
  }

//   Animation that fades all the LEDs of a strip simultaneously between Red, Green, and Blue
  public void setRGBFadeAnimation (int frameRate, int slot){
    candle.setControl(
      new RgbFadeAnimation(0, kLED.numOfLED + 7)
        .withFrameRate(frameRate)
        .withSlot(slot)
    );
  }

//   Animation that fades into and out of a specified color
  public void setSingleFadeAnimation (int r, int g, int b, int frameRate, int slot){
    candle.setControl(
      new SingleFadeAnimation(0, kLED.numOfLED + 7)
        .withColor(new RGBWColor(r, g, b))
        .withFrameRate(frameRate)
        .withSlot(slot)
    );
  }

//   Animation that strobes the LEDs a specified color
  public void setStrobeAnimation(int r, int g, int b, int frameRate, int slot) {
    candle.setControl(
      new StrobeAnimation(0, kLED.numOfLED + 7)
        .withColor(new RGBWColor(r, g, b))
        .withFrameRate(frameRate)
        .withSlot(slot)
    );
  }

// Sets the current LED mode
  public void setLEDMode(LEDModes inputLEDMode) {
    LEDMode = inputLEDMode;
  }
// Gets the current LED mode
  public LEDModes getLEDMode() {
    return LEDMode;
  }
// Command to set the LED mode
  public Command setLEDModeCommand(LEDModes inputLEDMode) {
    return Commands.runOnce(() -> this.setLEDMode(inputLEDMode)).ignoringDisable(true);
  }
// Command to get the LED mode
  public Command getLEDModeCommand() {
    return Commands.runOnce(() -> this.getLEDMode());
  }
}
