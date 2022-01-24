// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LED extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.LED.CANdleID, "FastFD");
    private final int LedCount = 300;
    private AnimationTypes m_currentAnimation;
    int red = 0;
    int green = 0;
    int blue = 0;

    private Animation m_toAnimate = null;

    public LED() {
        // Setup LED strip
        setPattern(8,95,0,255, 1, AnimationTypes.Solid);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    /**
     * Set the LEDs with a color and animation type
     *
     * @param red   the red value of the color
     * @param green the green value of the color
     * @param blue  the blue value of the color
     * @param white the white value of the color
     * @param speed the speed of the animation
     * @param toChange the animation mode of the LEDs
     */
    public void setPattern(int red, int green, int blue, int white, double speed, AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch (toChange) {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(red, green, blue, white, speed, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson: // a line bouncing back and forth with its width determined by size
                m_toAnimate = new LarsonAnimation(red, green, blue, white, 1, LedCount, BounceMode.Front, 3);
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
                m_toAnimate = new TwinkleAnimation(red, green, blue, white, speed, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(red, green, blue, white, speed, LedCount, TwinkleOffPercent.Percent100);
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
        System.out.println("Changed to " + m_currentAnimation.toString());
    }
    /**
     * Interpret a robot state and set the LEDs to express that state
     *
     * @param state the dominant robot state that the LEDs will express
     */
    public void expressState(robotState state) {
        switch (state) {
            case Shooting: // Strobing green
                setPattern(58,199,71,0,0.1,AnimationTypes.Strobe);
                break;
            case Intaking: // Solid Blue
                setPattern(66,95,255,0,0,AnimationTypes.Solid);
                break;
            case Idle: // Fuchsia larson animation
                setPattern(255,0,255,0,0.1,AnimationTypes.Larson);
                break;
            case Climbing: // cyceling rainbow
                setPattern(0,0,0,0,0.4,AnimationTypes.Rainbow);
                break;
            case Disabled: // solid red
                setPattern(255,0,0,0,0,AnimationTypes.Solid);
                break;
            case VisionLock: // strobing yellow
                setPattern(255,255,153,0,0.1,AnimationTypes.Strobe);
                break;
            default:
                setPattern(106, 90, 205, 0, 0.4,AnimationTypes.Twinkle);
                break;
        }
    }

    @Override
    public void periodic() {
        if(m_toAnimate == null) {
            m_candle.setLEDs(red,green,blue);
        } else {
            m_candle.animate(m_toAnimate);
        }
    }

    /**
     * Different LED animation types
     */
    public enum AnimationTypes {
        ColorFlow, Fire, Larson, Rainbow, RgbFade, SingleFade, Strobe, Twinkle, TwinkleOff, Solid
    }

    /**
     * Different robot states
     */
    public enum robotState {
        Climbing, Disabled, Idle, Intaking, Shooting, VisionLock
    }
}
