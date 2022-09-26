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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** create a new LED subsystem */
public class LED extends SubsystemBase {
  private final CANdle m_candle = new CANdle(Constants.LED.CANdleID);
  int red = 0;
  int green = 0;
  int blue = 0;
  private robotState currentRobotState = robotState.Disabled;
  private Animation m_toAnimate = null;

  private final Controls m_controls;

  private final int ledCount = 296;

  public LED(Controls controls) {
    // Setup LED strip
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.GRB;
    configAll.brightnessScalar = 1; // 1 is highest safe value
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_candle.configAllSettings(configAll, 100);

    m_controls = controls;
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
  }

  /**
   * Interpret a robot state and set the LEDs to express that state
   *
   * @param state the dominant robot state that the LEDs will express
   */
  public void expressState(robotState state) {
    if (state != currentRobotState) {
      switch (state) {
        case Intaking: // Solid Yellow
          setPattern(255, 128, 0, 0, 0, AnimationTypes.Solid);
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
        case CanShoot: // Solid Blue
          setPattern(66, 95, 255, 0, 0, AnimationTypes.Solid);
          break;
        case OpponentBall: // Returns Wrong Color
          switch (m_controls.getAllianceColor()) {
            case Blue:
              setPattern(255, 0, 170, 0, 0, AnimationTypes.Solid); // Solid Pink
              break;
            case Red:
              setPattern(0, 255, 255, 0, 0, AnimationTypes.Solid); // Solid Teal
              break;
            case Invalid:
            default:
              setPattern(255, 0, 255, 0, 1, AnimationTypes.Strobe); // Strobing Purple
              break;
          }
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
      m_candle.setLEDs(255, 30, 0, 0, 0, 125);
      m_candle.setLEDs(red, green, blue, 0, 20, 35); // setting all LEDs to color
    } else {
      m_candle.animate(m_toAnimate); // setting the candle animation to m_animation if not null
    }
    SmartDashboardTab.putString("Controls", "LED Mode", currentRobotState.toString());
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
    CanShoot,
    OpponentBall
  }
}
