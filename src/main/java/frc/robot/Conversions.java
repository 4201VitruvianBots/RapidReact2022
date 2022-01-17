package frc.robot;

public class Conversions {
    public static double RpmToRadPerSec(double rpm) {
        return rpm * ((2 * Math.PI) / 60.0);
    }

    public static double RadPerSecToRpm(double radPerSec) {
        return radPerSec * (60.0 / (2 * Math.PI));
    }
}
