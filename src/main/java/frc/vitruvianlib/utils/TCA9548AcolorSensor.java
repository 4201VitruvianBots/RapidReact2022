package frc.vitruvianlib.utils;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;

public class TCA9548AcolorSensor extends I2C {
  private Port port;
  private int address = 0x70;
  private int channel;

  private ColorSensorV3 frontColorSensor;
  private ColorSensorV3 rearColorSensor;
  private ColorSensorV3 colorSensor;

  public TCA9548AcolorSensor(Port port) {
    super(port, 0x70);
    this.port = port;

    frontColorSensor = new ColorSensorV3(port);
    rearColorSensor = new ColorSensorV3(port);

    selectMuxChannel(0);
  }

  public void selectMuxChannel(int channel) {
    this.channel = channel;
    write(0, 1 << channel);
    this.colorSensor =
        (channel == Constants.Indexer.colorSensorFront) ? frontColorSensor : rearColorSensor;
  }

  public int getMuxChannel() {
    return channel;
  }

  public ColorSensorV3 getColorSensor() {
    return colorSensor;
  }
}
