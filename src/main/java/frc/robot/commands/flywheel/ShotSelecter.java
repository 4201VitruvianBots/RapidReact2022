// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

public class ShotSelecter {
  private static final ShotRecipe[] shotRecipes = {
    new ShotRecipe(1650, .8, 3.08),
    new ShotRecipe(1700, .8, 3.15),
    new ShotRecipe(1780, .8, 3.25),
    new ShotRecipe(1900, .8, 3.44),
    new ShotRecipe(2000, .8, 3.65),
  };

  public ShotSelecter() {
  }
  //function that takes distance and returns the rpm of the shot with the closest distance
  public static ShotRecipe bestShot(double distance) {
    //return shot with closest distance
    ShotRecipe closest = shotRecipes[0];
    for (ShotRecipe shot : shotRecipes) {
      if (Math.abs(shot.getDistance() - distance) < Math.abs(closest.getDistance() - distance)) {
        closest = shot;
      }
    }
    return closest;
  }
}