// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
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
  private final int LedCount = 8 + 144 + 144;
  private AnimationTypes m_currentAnimation;
  private robotState currentRobotState;
  int red = 0;
  int green = 0;
  int blue = 0;

  private int LEDTestingIndex = 0;

  private Animation m_toAnimate = null;

  public LED() {
    // Setup LED strip
    setPattern(0, 255, 0, 255, 1, AnimationTypes.Solid);
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 0.1; // normal scalar is 0.1 || dimmed scalar is 0.01
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);
  }

  /**
   * Set the LEDs with a color and animation type
   *
   * @param red      the red value of the color
   * @param green    the green value of the color
   * @param blue     the blue value of the color
   * @param white    the white value of the color
   * @param speed    the speed of the animation
   * @param toChange the animation mode of the LEDs
   */
  public void setPattern(
      int red, int green, int blue, int white, double speed, AnimationTypes toChange) {
    m_currentAnimation = toChange;

    switch (toChange) {
      case ColorFlow:
        m_toAnimate = new ColorFlowAnimation(red, green, blue, white, speed, LedCount, Direction.Forward);
        break;
      case Fire:
        m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
        break;
      case Larson: // a line bouncing back and forth with its width determined by size
        m_toAnimate = new LarsonAnimation(red, green, blue, white, 0.001, LedCount, BounceMode.Front, 1);
        break;
      case Rainbow: // neon cat type beat
        m_toAnimate = new RainbowAnimation(1, speed, LedCount);
        break;
      case RgbFade: // cycling between red, greed, and blue
        m_toAnimate = new RgbFadeAnimation(1, speed, LedCount);
        break;
      case SingleFade:
        m_toAnimate = new SingleFadeAnimation(red, green, blue, white, speed, LedCount);
        break;
      case Strobe: // ahhhhhhhhhhhhhh
        m_toAnimate = new StrobeAnimation(red, green, blue, white, speed, LedCount);
        break;
      case Twinkle:
        m_toAnimate = new TwinkleAnimation(red, green, blue, white, speed, LedCount, TwinklePercent.Percent100);
        break;
      case TwinkleOff:
        m_toAnimate = new TwinkleOffAnimation(
            red, green, blue, white, speed, LedCount, TwinkleOffPercent.Percent100);
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
    if (m_toAnimate == null) {
      System.out.println("Changed to solid (" + red + ", " + green + ", " + blue + ")");
    } else {
      System.out.println("Changed to " + m_currentAnimation.toString());
    }
  }

  public void increaseTestingState(boolean increaseState) {
    if (increaseState) {
      LEDTestingIndex++;
    } else {
      LEDTestingIndex--;
    }
    expressTestingState();
  }

  public void expressTestingState() {
    // int testingState = LEDTestingIndex % 5;
    // switch (testingState) {
    // case 0: // strobing yellow
    // setPattern(255, 255, 0, 0, 1, AnimationTypes.Strobe);
    // break;
    // case 1: // Green larson animation
    // setPattern(0, 255, 0, 0, 0.1, AnimationTypes.Larson);
    // break;
    // case 2: // Rainbow
    // setPattern(0, 0, 0, 0, 0.4, AnimationTypes.Rainbow);
    // break;
    // case 3: // solid red
    // setPattern(255, 0, 0, 0, 0, AnimationTypes.Solid);
    // break;
    // case 4: // strobing blue
    // setPattern(50, 0, 255, 0, 0, AnimationTypes.Solid);
    // break;
    // default:
    // setPattern(0, 0, 0, 0, 0, AnimationTypes.Solid);
    // break;
    // }
    setPattern(50, 0, 255, 0, 0, AnimationTypes.Solid);
  }

  @Override
  public void periodic() {
    if
    if (m_toAnimate == null) {
      // m_candle.setLEDs(0, 0, 0, 0, 512, 1024);
      // m_candle.setLEDs(0, 0, 0, 0, 0, 512);
      // System.out.println("all off");

      m_candle.setLEDs(red, green, blue, 0, 512, 1024);
      m_candle.setLEDs(red, green, blue, 0, 0, 512);
      System.out.println("all on");
    } else {
      m_candle.animate(m_toAnimate);
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
    VisionLock
  }
}
