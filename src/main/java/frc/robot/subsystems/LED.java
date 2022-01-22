// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
Subsystem for controlling robot LEDs
 */
public class LED extends SubsystemBase {
  private final AddressableLEDBuffer LEDBuffer;

  final int stripLength;
  double hueOffset = 0;
  int head = 0;

  public LED() {
    // Setup LED strip
    AddressableLED LEDStrip = new AddressableLED(frc.robot.Constants.LED.ledPort);
    LEDBuffer = new AddressableLEDBuffer(111);
    LEDStrip.setLength(LEDBuffer.getLength());
    LEDStrip.setData(LEDBuffer);
    LEDStrip.start();
    stripLength = LEDBuffer.getLength() / 2;
  }

  /**
   * Set the LEDs with a color and lighting mode
   *
   * @param red the red value of the color
   * @param green the green value of the color
   * @param blue the blue value of the color
   * @param mode the lighting mode of the LEDs
   */
  public void setLED(int red, int green, int blue, LEDMode mode) {
    switch (mode) {
      case BLINK:
        double time = (int) (5 * Timer.getFPGATimestamp());
        if (time / 2 == Math.floor(time / 2)) {
          for (int i = 0; i < stripLength; i++) {
            LEDBuffer.setRGB(i, red, green, blue);
            LEDBuffer.setRGB(stripLength + i, red, green, blue);
          }
        } else setLED(LEDMode.OFF);
        break;
      case TWINKLE:
        setLED(LEDMode.OFF);
        for (int i = head; i < LEDBuffer.getLength(); i += 2) {
          LEDBuffer.setRGB(i % LEDBuffer.getLength(), red, green, blue);
        }
        Timer.delay(0.2);
        head = ++head % 2;
        break;
      case OFF:
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
          LEDBuffer.setRGB(i, 0, 0, 0);
        }
        break;
      case RAINBOW:
        for (int i = 0; i < stripLength; i++) {
          LEDBuffer.setHSV(i, (int) (900 * i / stripLength + hueOffset) % 180, 255, 255);
          LEDBuffer.setHSV(
              stripLength + i, (int) (900 * i / stripLength + hueOffset) % 180, 255, 255);
        }
        hueOffset = (hueOffset + 120) % 180;
        Timer.delay(0.05);
        break;
      case SOLID:
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
          LEDBuffer.setRGB(i, (int) (red * 0.5), (int) (blue * 0.5), (int) (green * 0.5));
        }
        break;
      case TRAILING:
        for (int i = head; i < LEDBuffer.getLength(); i += 5) {
          LEDBuffer.setRGB(
              i % LEDBuffer.getLength(),
              (int) (red * 0.5),
              (int) (blue * 0.5),
              (int) (green * 0.5));
        }
        Timer.delay(0.03);
        head = ++head % 5;
        break;
    }
  }

  /**
   * Set the LEDs with a color and lighting mode
   *
   * @param mode the lighting mode of the LEDs
   */
  public void setLED(LEDMode mode) {
    setLED(0, 0, 0, mode);
  }

  /**
   * Interpret a robot state and set the LEDs to express that state
   *
   * @param state the dominant robot state that the LEDs will express
   */
  public void expressState(robotState state) {
    // Rainbow
    // Blinking dirty-yellow
    // Solid red
    // Blinking purple
    switch (state) {
      case READY:
        setLED(LEDMode.RAINBOW);
        break;
      case SET:
        setLED(255, 200, 0, LEDMode.TWINKLE);
        break;
      case GO:
        setLED(255, 0, 0, LEDMode.SOLID);
        break;
      default:
        setLED(106, 90, 205, LEDMode.TWINKLE);
        break;
    }
  }

  @Override
  public void periodic() {}

  /** Different LED modes */
  public enum LEDMode {
    BLINK, // flashing on and off
    TWINKLE, // alternating even and odd LEDs
    OFF, // all LEDs off
    RAINBOW, // a trailing rainbow
    SOLID, // one solid color
    TRAILING // snake of colour cycling through LEDs
  }

  /** Different robot states */
  public enum robotState {
    READY,
    SET,
    GO,
    NOPE
  }
}
