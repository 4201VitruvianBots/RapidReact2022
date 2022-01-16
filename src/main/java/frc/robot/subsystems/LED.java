// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
Subsystem for controlling robot LEDs
 */
//TODO: rewrite LED subsystem with comments aswell as redesigning the assigned colour values for each state

public class LED extends SubsystemBase {
    private final AddressableLED LEDStrip;
    private final AddressableLEDBuffer LEDBuffer;
    int start = (int) Timer.getFPGATimestamp() * 5;

    int stripLength;
    double rainbows = 3;
    double speed = 8;
    double hueOffset = 0;
    int head = 0;
    int offset = 0;
    int cRed = 0;
    int cGreen = 0;
    int cBlue = 0;
    int realPanelColor = 0;
    int state = -1;
    private int red, green, blue;

    public LED() {
        // Setup LED strip
        // TODO: make sure that the LEDs that we are using this year corilign with the
        // code
        LEDStrip = new AddressableLED(frc.robot.Constants.LED.ledPort);
        LEDBuffer = new AddressableLEDBuffer(111);
        LEDStrip.setLength(LEDBuffer.getLength());
        LEDStrip.setData(LEDBuffer);
        LEDStrip.start();
        stripLength = LEDBuffer.getLength() / 2;

        red = 0;
        green = 125;
        blue = 0;
        setSolidColor();
        // SmartDashboard.putNumber("Rainbows", rainbows);
        // SmartDashboard.putNumber("Speed", speed);

    }

    // Set LED color based on RGB value
    public void setRGB(int red, int green, int blue) {
        this.red = (int) (red * 0.5);
        this.blue = (int) (blue * 0.5);
        this.green = (int) (green * 0.5);
    }

    public void setSolidColor() {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, red, green, blue);
        }
    }

    public void resetLED() {
        for (int i = 0; i < LEDBuffer.getLength(); i++) {
            LEDBuffer.setRGB(i, 0, 0, 0);
        }
    }

    /**
     * Sets LED colors
     *
     * @param blinkType whether to set the LEDs to blink
     */
    public void setBlinkingColor(boolean blinkType) {
        double time = (int) (5 * Timer.getFPGATimestamp());
        if (!blinkType) {
            if (time / 2 == Math.floor(time / 2)) {
                for (int i = 0; i < stripLength; i++) {
                    LEDBuffer.setRGB(i, red, green, blue);
                    LEDBuffer.setRGB(stripLength + i, red, green, blue);
                }
            } else resetLED();
        } else {
            resetLED();
            for (int i = head; i < LEDBuffer.getLength(); i += 2) {
                LEDBuffer.setRGB(i % LEDBuffer.getLength(), red, green, blue);
            }
            Timer.delay(0.2);
            head = ++head % 2;
        }
    }

    // TODO: simplify rainbow
    public void setRainbow(double iterations, double speed) {
        for (int i = 0; i < stripLength; i++) {
            LEDBuffer.setHSV(i, (int) (180 * iterations * i / stripLength + hueOffset) % 180, 255, 255);
            LEDBuffer.setHSV(stripLength + i, (int) (180 * iterations * i / stripLength + hueOffset) % 180, 255, 255);
        }
        hueOffset = (hueOffset + 3 * speed * iterations) % 180;
        Timer.delay(0.05);
    }

    public void trail(int interval) {
        resetLED();
        for (int i = head; i < LEDBuffer.getLength(); i += interval) {
            LEDBuffer.setRGB(i % LEDBuffer.getLength(), red, green, blue);
        }
        Timer.delay(0.03);
        head = ++head % interval;
    }

    public void flash() {
        setSolidColor();
        Timer.delay(0.1);
        resetLED();
        Timer.delay(0.1);
        setSolidColor();
        Timer.delay(0.25);
    }

    public void colorToRGB(int color) {
        switch (color) {
            case 1:
                setRGB(255, 0, 0);
                break;
            case 2:
                setRGB(0, 255, 0);
                break;
            case 3:
                setRGB(0, 255, 255);
                break;
            case 4:
                setRGB(255, 255, 0);
                break;
        }
    }

    // TODO: rewrite this for teh correct states, talk to the team about what states
    // they want and what colors
    public void setLED() {
        switch (state) {
            case 0:
                setRainbow(3, 8);
                break;
            case 1:
                setRGB(255, 200, 0); // Blinking dirty-yellow
                setBlinkingColor(true);
                break;
            case 2:
                setRGB(255, 0, 0); // Solid red
                setSolidColor();
                break;
            case 3:
                setRGB(255, 255, 255); // Flashing white
                flash();
                break;
            case 4:
                setRGB(66, 194, 23); // Trailing vitruvian green
                trail(5);
                break;
            case 5:
                setRGB(0, 110, 255); // Blinking dark blue
                setBlinkingColor(true);
                break;
            case 6:
                setRGB(255, 110, 0); // Trailing reddish-orange
                trail(8);
                break;
            case 7:
                setRGB(255, 0, 0); // Blinking red
                setBlinkingColor(true);
                break;
            case 8:
                setRGB(0, 255, 0); // Blinking green
                setBlinkingColor(true);
                break;
            default:
                setRGB(106, 90, 205); // Blinking purple
                setBlinkingColor(true);
        }
        LEDStrip.setData(LEDBuffer);
    }

    public void setState(int state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // setRainbow(rainbows, speed);
        /*
         * rainbows = SmartDashboard.getNumber("Rainbows", 0);
         * speed = SmartDashboard.getNumber("Speed", 0);
         * setBuffer();
         */
    }

    public void ledPeriodic() {
        if (RobotBase.isReal()) {
            setLED();
            LEDStrip.setData(LEDBuffer);
        }
    }
}
