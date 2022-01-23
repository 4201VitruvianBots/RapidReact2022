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
import frc.robot.Constants;
import frc.robot.RobotContainer;

/*
Code to interact with the robot's color sensor for the wheel of fortune (WoF)
 */

public class ColorSensors extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // setup variables
  public boolean practiceField = false;

  public boolean isColor = false;
  public boolean working = false;
  public int semiRotations = 0;
  public int realIntervals = 0;
  public ColorSensorV3 colorTopSensor = new ColorSensorV3(I2C.Port.kOnboard);
  public ColorSensorV3 colorBottomSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private int colorIDTop;
  private int colorIDBottom;
  public boolean blueTeamBall;
  public int allianceColor;

  public Color getTopColor() {
    return colorTopSensor.getColor();
  }

  public Color getBottomColor() {
    return colorBottomSensor.getColor();
  }

  // ???
  public double getTopIR() {
    return colorTopSensor.getIR();
  }

  public double getBottomIR() {
    return colorBottomSensor.getIR();
  }

  // get distance to color
  public int getTopProximity() {
    return colorTopSensor.getProximity();
  }

  public int getBottomProximity() {
    return colorBottomSensor.getProximity();
  }
 
  // ???
  public int panelTopColor() { // none = 0; red = 1; blue = 2;
    if (practiceField) {
      if (getTopColor().red > getTopColor().green && getTopColor().green * 1.8 > getTopColor().red) {
        return 1;

      }
      if (getTopColor().blue * 1.25 > getTopColor().green && getTopColor().blue * 1.02 < getTopColor().green && getTopColor().blue > getTopColor().red * 1.89) {
        return 2;

      } else return 0;
    } 

    else {

      if (getTopColor().red > getTopColor().blue * 3 && getTopColor().red > getTopColor().green * 1.33) {
        return 1;
      } else if (getTopColor().blue < getTopColor().green * 1.15 && getTopColor().green < getTopColor().blue * 1.15 && getTopColor().blue > getTopColor().red * 2.5) {
        return 2;
      } else return 0;
    }

  }


  /**
   * sets the alliance color depending on the alliance selection on Shuffleboard (robotContainer)
   * @return
   */
  public int allianceSelection(){
    if(RobotContainer.allianceColorBlue == true) {
      int allianceColor = 2;
      return allianceColor;
    }else if(RobotContainer.allianceColorRed == true) {
      int allianceColor = 1;
      return allianceColor;
    }else{
      return allianceColor = 0;

    }

  }

  /**
   * Checks if the alliance is red
   */
  public boolean redMatch(){
    if( allianceColor == panelTopColor() && allianceColor == panelBottomColor()) {
      return true;
    }else{
      return false;
    }
  }

   /**
   * Checks if the alliance is red
   */
  public boolean blueMatch(){
    if( allianceColor == panelTopColor() && allianceColor == panelBottomColor()) {
      return true;
    }else{
      return false;
    }
  }



  public int panelBottomColor() { // none = 0; red = 1; green = 2; blue = 3; yellow = 4   if red is greater than Green, and Green * 1.8 is greater than red
    if (practiceField) {
      if (getBottomColor().red > getBottomColor().green && getBottomColor().green * 1.8 > getBottomColor().red) {
        return 1;
      }
      if (getBottomColor().blue * 1.25 > getBottomColor().green && getBottomColor().blue * 1.02 < getBottomColor().green && getBottomColor().blue > getBottomColor().red * 1.89) {
        return 2;
      }
        else 
          return 0;

    } 
    
    else {

      if (getBottomColor().red > getBottomColor().blue * 3 && getBottomColor().red > getBottomColor().green * 1.33) {
        return 1;
      } 
      else if (getBottomColor().blue < getBottomColor().green * 1.15 && getBottomColor().green < getBottomColor().blue * 1.15 && getBottomColor().blue > getBottomColor().red * 2.5) {
        return 2;
      
      } else return 0;
    }
  }

  public void resetRotationControlVars() {
    isColor = true;
    semiRotations = 0;
    realIntervals = 0;
    colorIDTop = panelTopColor();
    colorIDBottom = panelBottomColor();
  } // Resets RotationControlVars when method is called upon

  public boolean rotationControlComplete() {

    if (panelTopColor() != colorIDTop && panelTopColor() != 0) {
      colorIDTop = panelTopColor();
      realIntervals++;
    }

    if (panelBottomColor() != colorIDBottom && panelBottomColor() != 0) {
      colorIDBottom = panelBottomColor();
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
    SmartDashboard.putNumber("Red", getTopColor().red);
    SmartDashboard.putNumber("Blue", getTopColor().blue);
    SmartDashboard.putNumber("Red", getBottomColor().red);
    SmartDashboard.putNumber("Blue", getBottomColor().blue);
    SmartDashboard.putNumber("IR", getTopIR());
    SmartDashboard.putNumber("IR", getBottomIR());
    SmartDashboard.putNumber("Proximity", getTopProximity());
    SmartDashboard.putNumber("Proximity", getBottomProximity());
    switch (panelTopColor()) {
      case 1:
        colorName = "Red";
        break;
      case 2:
        colorName = "Blue";
        break;
    }

    switch (panelBottomColor()) {
      case 1:
        colorName = "Red";
        break;
      case 2:
        colorName = "Blue";
        break;
    }
    SmartDashboard.putBoolean("Red", panelTopColor() == 1);
    SmartDashboard.putBoolean("Blue", panelTopColor() == 2);
    SmartDashboard.putBoolean("Red", panelBottomColor() == 1);
    SmartDashboard.putBoolean("Blue", panelBottomColor() == 2);
    SmartDashboard.putBoolean("Rotation Control Complete", rotationControlComplete());
    SmartDashboard.putNumber("Real Intervals", realIntervals);
    // SmartDashboard.putString("Color", getColorString());
    SmartDashboardTab.putNumber("ColorSensor", "Panel Color", panelTopColor());
    SmartDashboardTab.putNumber("ColorSensor", "Panel Color", panelBottomColor());
    SmartDashboardTab.putBoolean("ColorSensor", "Rotation Control Complete", rotationControlComplete());
    SmartDashboardTab.putNumber("ColorSensor", "Semi Rotations", semiRotations);

    SmartDashboard.putNumber("Thing", getFMSColor());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateSmartDashboard();
  }
}
