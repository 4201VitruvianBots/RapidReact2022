/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
Code to interact with the robot's color sensor for the wheel of fortune (WoF)
 */

public class ColorSensor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // setup variables
  public boolean practiceField = false;

  public boolean isColor = false;
  public boolean working = false;
  public int semiRotations = 0;
  public int realIntervals = 0;
  public ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kOnboard);
  private int colorID;

  public Color getColor() {
    return sensor.getColor();
  }

  // ???
  public double getIR() {
    return sensor.getIR();
  }

  // get distance to color
  public int getProximity() {
    return sensor.getProximity();
  }

  // ???
  public int panelColor() { // none = 0; red = 1; green = 2; blue = 3; yellow = 4
    if (practiceField) {
      if (getColor().red > getColor().green && getColor().green * 1.8 > getColor().red) {
        return 1;
      }
      if (getColor().red * 2.26 < getColor().green && getColor().blue * 2.16 < getColor().green) {
        return 2;
      }
      if (getColor().blue * 1.25 > getColor().green
          && getColor().blue * 1.02 < getColor().green
          && getColor().blue > getColor().red * 1.89) {
        return 3;
      }
      if (getColor().red * 1.83 > getColor().green
          && getColor().red * 1.1 < getColor().green
          && getColor().red > getColor().blue * 1.86) {
        return 4;
      } else return 0;
    } else {
      if (getColor().red > getColor().blue * 3 && getColor().red > getColor().green * 1.33) {
        return 1;
      } else if (getColor().green > getColor().red * 2.75
          && getColor().green > getColor().blue * 1.8) {
        return 2;
      } else if (getColor().blue < getColor().green * 1.15
          && getColor().green < getColor().blue * 1.15
          && getColor().blue > getColor().red * 2.5) {
        return 3;
      } else if (getColor().green < getColor().red * 1.8
          && getColor().green > getColor().red * 1.65) {
        return 4;
      } else return 0;
    } // Determines the displayed color based on the quantative amount of said color???????
  }

  public void resetRotationControlVars() {
    isColor = true;
    semiRotations = 0;
    realIntervals = 0;
    colorID = panelColor();
  } // Resets RotationControlVars when method is called upon

  public boolean rotationControlComplete() {
    /*if(panelColor() == colorID && isColor == false){
      isColor = true;
      semiRotations++;
    } else if(panelColor() != colorID && isColor){
      isColor = false;
    }
    if(semiRotations >= 6 && isColor == false){
      return true;
    } else
      return false;*/
    if (panelColor() != colorID && panelColor() != 0) {
      colorID = panelColor();
      realIntervals++;
    }
    return realIntervals > 24;
  }

  public int getFMSColor() {
    String message = DriverStation.getInstance().getGameSpecificMessage();
    switch (message) {
      case "R":
        return 1;
      case "G":
        return 2;
      case "B":
        return 3;
      case "Y":
        return 4;
      default:
        return -1;
    }
  } // returns a value that corresponds with a color based on the message

  // Sets smartdashboard
  public void updateSmartDashboard() {
    String colorName = "Not Close Enough";
    SmartDashboard.putNumber("Red", getColor().red);
    SmartDashboard.putNumber("Green", getColor().green);
    SmartDashboard.putNumber("Blue", getColor().blue);
    SmartDashboard.putNumber("IR", getIR());
    SmartDashboard.putNumber("Proximity", getProximity());
    switch (panelColor()) {
      case 1:
        colorName = "Red";
        break;
      case 2:
        colorName = "Green";
        break;
      case 3:
        colorName = "Blue";
        break;
      case 4:
        colorName = "Yellow";
        break;
    }
    SmartDashboard.putBoolean("Red", panelColor() == 1);
    SmartDashboard.putBoolean("Green", panelColor() == 2);
    SmartDashboard.putBoolean("Blue", panelColor() == 3);
    SmartDashboard.putBoolean("Yellow", panelColor() == 4);
    SmartDashboard.putBoolean("Rotation Control Complete", rotationControlComplete());
    SmartDashboard.putNumber("Real Intervals", realIntervals);
    // SmartDashboard.putString("Color", getColorString());
    SmartDashboardTab.putNumber("ColorSensor", "Panel Color", panelColor());
    SmartDashboardTab.putBoolean(
        "ColorSensor", "Rotation Control Complete", rotationControlComplete());
    SmartDashboardTab.putNumber("ColorSensor", "Semi Rotations", semiRotations);

    SmartDashboard.putNumber("Thing", getFMSColor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateSmartDashboard();
  }
}
