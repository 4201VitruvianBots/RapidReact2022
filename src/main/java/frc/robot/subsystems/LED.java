// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** create a new LED subsystem */
public class LED extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.LED.CANdleID);
  int red = 0;
  int green = 0;
  int blue = 0;
  private robotState currentRobotState;
  private Animation m_toAnimate = null;

  public LED() {
    // Setup LED strip
    setPattern(8, 95, 0, 255, 1, AnimationTypes.Solid);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1; // 1 is highest safe value
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  /**
   * Set the LEDs with a color and animation type
   *
   * @param red the red value of the color
   * @param green the green value of the color
   * @param blue the blue value of the color
   * @param white the white value of the color
   * @param speed the speed of the animation
   * @param toChange the animation mode of the LEDs
   */
  public void setPattern(
      int red, int green, int blue, int white, double speed, AnimationTypes toChange) {

    int ledCount = 296;
    switch (toChange) {
      case ColorFlow: // stripe of color flowing through the led strip
        m_toAnimate =
            new ColorFlowAnimation(red, green, blue, white, speed, ledCount, Direction.Forward);
        break;
      case Fire: // red and orange leds flaming up and down the led strip
        m_toAnimate = new FireAnimation(0.5, 0.7, ledCount, 0.7, 0.5);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate =
            new LarsonAnimation(red, green, blue, white, speed, ledCount, BounceMode.Front, 7);
        break;
      case Rainbow: // neon cat type beat
        m_toAnimate = new RainbowAnimation(1, speed, ledCount);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_toAnimate = new RgbFadeAnimation(1, speed, ledCount);
        break;
      case SingleFade: // slowly turn all leds from solid color to off
        m_toAnimate = new SingleFadeAnimation(red, green, blue, white, speed, ledCount);
        break;
      case Strobe: // switching between solid color and full off at high speed
        m_toAnimate = new StrobeAnimation(red, green, blue, white, speed, ledCount);
        break;
      case Twinkle: // random leds turning on and off with certain color
        m_toAnimate =
            new TwinkleAnimation(red, green, blue, white, speed, ledCount, TwinklePercent.Percent6);
        break;
      case TwinkleOff: // twinkle in reverse
        m_toAnimate =
            new TwinkleOffAnimation(
                red, green, blue, white, speed, ledCount, TwinkleOffPercent.Percent100);
        break;
      case Solid:
        this.red = red;
        this.green = green;
        this.blue = blue;
        m_toAnimate = null;
        break;
      default:
        System.out.println("Incorrect animation type provided to changeAnimation() method");
    }
    // if (m_toAnimate == null) {
    // System.out.println("Changed to solid (" + red + ", " + green + ", " + blue +
    // ")");
    // } else {
    // System.out.println("Changed to " + toChange);
    // }
  }

  /**
   * Interpret a robot state and set the LEDs to express that state
   *
   * @param state the dominant robot state that the LEDs will express
   */
  public void expressState(robotState state) {
    if (state != currentRobotState) {
      switch (state) {
        case Intaking: // Solid Blue
          setPattern(255, 255, 0, 0, 1, AnimationTypes.Strobe);
          break;
        case Enabled: // Solid Green
          setPattern(0, 255, 0, 0, 0, AnimationTypes.Solid);
          break;
        case Climbing: // Rainbow
          setPattern(0, 0, 0, 0, .9, AnimationTypes.Rainbow);
          break;
        case Disabled: // Solid Red
          setPattern(255, 0, 0, 0, 0, AnimationTypes.Solid);
          break;
        case CanShoot: // Strobing Yellow
          setPattern(66, 95, 255, 0, 0, AnimationTypes.Solid);
          break;
        default: // Strobing Purple
          setPattern(255, 0, 255, 0, 1, AnimationTypes.Strobe);
          break;
      }
      currentRobotState = state;
    }
  }

  @Override
  public void periodic() {
    // null indicates that the animation is "Solid"
    if (m_toAnimate == null) {
      m_candle.setLEDs(red, green, blue, 0, 0, 1024); // setting all LEDs to color
    } else {
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }
  }

  /** Different LED animation types */
  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    Solid
  }

  /** Different robot states */
  public enum robotState {
    Climbing,
    Disabled,
    Enabled,
    Intaking,
    CanShoot
  }
}
