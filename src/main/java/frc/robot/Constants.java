// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

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
        public static final int indexerMotor = 35;
        public static final int indexerTopSensor = 1;
        public static final int indexerBottomSensor = 2;
    }

    public final class Intake {
        public static final int pcmOne = 11;
        public static final int intakePistonForward = 0;
        public static final int intakePistonReverse = 1;
        public static final int intakeMotor = 47;
        public static final int intakeSensor = 0;
    }

    public final class LED {

    }

    public final class Outtake {

    }

    public final class Vision {

    }

}
