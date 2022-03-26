// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

public class ShotSelecter {
  private static final ShotRecipe[] shotRecipes = {
    new ShotRecipe(0, 0, 0),
    new ShotRecipe(0, 0, 0),
    new ShotRecipe(0, 0, 0),
    new ShotRecipe(0, 0, 0),
    new ShotRecipe(0, 0, 0),
    new ShotRecipe(0, 0, 0),
    new ShotRecipe(0, 0, 0)
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