// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class USB {
        public static final int leftJoystick = 0;
        public static final int rightJoystick = 1;
        public static final int xBoxController = 2;
    }

    public final class Climber {

    }

    public final class DriveTrain {

    }

    public final class Indexer {

    }

    public final class Intake {

    }

    public final class LED {

    }

    public final class Outtake {
        public static final int encoderUnitsPerRotation = 2048;

        // Volts per (radian per second)
        public static final double kFlywheelKv = 0.111;

        // Volts per (radian per second squared)
        public static final double kFlywheelKa = 0.02;

        public static final double rpmTolerance = 50.0;

        public static final int flywheelMotorA = 40;
        public static final int flywheelMotorB = 41;
    }

    public final class Turret {
        public static final int turretMotor = 60;
        public static final int turretEncoder = 61;

    }

    public final class Vision {

    }

}
