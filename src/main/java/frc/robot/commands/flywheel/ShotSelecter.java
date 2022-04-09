// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.flywheel;

public class ShotSelecter {

  private static final ShotRecipe[] shotRecipes = {
    new ShotRecipe(1650, .8, 3.61),
    new ShotRecipe(1700, .8, 4.15),
    new ShotRecipe(1780, .8, 4.86),
    new ShotRecipe(1900, .8, 5.2),
    new ShotRecipe(2000, .8, 6.36),
  };

  private static double interpolatedRPM = 1650;
  private static ShotRecipe closest = shotRecipes[0], lastClosest = shotRecipes[0];

  public ShotSelecter() {}
  // function that takes distance and returns the rpm of the shot with the closest distance
  public static double bestShot(double distance) {
    // return shot with closest distance
    for (ShotRecipe shot : shotRecipes) {
      if (Math.abs(shot.getDistance() - distance) < Math.abs(closest.getDistance() - distance)
          && Math.abs(shot.getDistance() - distance) * 2
              < Math.abs(lastClosest.getDistance() - distance)) {
        closest = shot;
      }
    }
    lastClosest = closest;
    return closest.getRPM();
  }

  public static double interpolateRPM(double distance) {
    distance =
        Math.min(
            distance, 4.5); // Vertex of parabola is at 4.1, don't want RPMs decreasing afterward
    if (distance > 0) {
      interpolatedRPM =
          Math.round(
                  Math.max(-381.7 * distance * distance + 3493.24 * distance - 5886.42, 1650)
                      / 25.0)
              * 25; // Minimum RPM of 1600
    }
    return interpolatedRPM;
  }
}
