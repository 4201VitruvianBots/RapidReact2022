package frc.robot;

public class ShotRecipe {
  private final double rpm;
  private final double kickerRPM;
  private final double distance;

  public ShotRecipe(double rpm, double kickerRPM, double distance) {
    this.rpm = rpm;
    this.kickerRPM = kickerRPM;
    this.distance = distance;
  }

  public double getRPM() {
    return rpm;
  }

  public double getKickerRPM() {
    return kickerRPM;
  }

  public double getDistance() {
    return distance;
  }
}
