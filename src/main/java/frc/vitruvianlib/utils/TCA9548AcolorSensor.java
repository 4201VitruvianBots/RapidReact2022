package frc.vitruvianlib.utils;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;

public class TCA9548AcolorSensor extends I2C {
    private Port port;
    private int address = 0x70;
    private int channel;

    private ColorSensorV3 colorSensor;

    public TCA9548AcolorSensor(Port port) {
        super(port, 0x70);
        this.port = port;

        selectMuxChannel(0);
    }

    public void selectMuxChannel(int channel) {
        this.channel = channel;
        write(0, 1 << channel);
        this.colorSensor = new ColorSensorV3(port);
    }

    public int getMuxChannel() {
        return channel;
    }

    public ColorSensorV3 getColorSensor() {
        return colorSensor;
    }
}
