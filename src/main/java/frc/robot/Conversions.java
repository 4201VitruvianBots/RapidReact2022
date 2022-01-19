package frc.robot;

public class Conversions {
  public static double RpmToRadPerSec(double rpm) {
    return rpm * ((2 * Math.PI) / 60.0);
  }

  public static double RadPerSecToRpm(double radPerSec) {
    return radPerSec * (60.0 / (2 * Math.PI));
  }

  public static double DegreestoRadPerSec(double degrees) {
    return degrees
        * 0.017453; // 0.017453 is how many radians per second are in one degree per second
  }
}
