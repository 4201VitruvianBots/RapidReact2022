// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Vision.*;

public class Vision extends SubsystemBase {

  private final NetworkTable goal_camera;
  private final NetworkTable intake_camera;

  private CAMERA_TYPE goal_camera_type = CAMERA_TYPE.OAK_D;


  /** Creates a new Vision Subsystem. */
  public Vision() {
    goal_camera = NetworkTableInstance.getDefault().getTable(goal_camera_type.toString().toLowerCase());
    intake_camera = NetworkTableInstance.getDefault().getTable("OAK-1_Intake");

    PortForwarder.add(5800, goalCameraIP, 5800);
    PortForwarder.add(5801, goalCameraIP, 5801);
    PortForwarder.add(5802, intakeCameraIP, 5800);
    PortForwarder.add(5803, intakeCameraIP, 5801);
  }

  /** Returns the robot's current alliance color
   *
   *  @return Returns the current alliance color.
   */
  public DriverStation.Alliance getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if(alliance != DriverStation.Alliance.Blue || alliance != DriverStation.Alliance.Red) {
      System.out.println("Vision Subsystem Error: Invalid Alliance Color");
    }

    return DriverStation.getAlliance();
  }

  /** Returns a boolean value for the goal target state
   *
   *  @return
   *  true: Goal Camera has a target.
   *  false: Goal Camera does not have a target.
   */
  public boolean getGoalValidTarget() {
    return goal_camera.getEntry("tv").getBoolean(false);
  }

  /** Returns the angle of the goal target in degrees.
   *
   *  @return
   *  Range: +/- 20 degrees
   */
  public double getGoalTargetXAngle() {
    return goal_camera.getEntry("tx").getDouble(0);
  }

  /** Returns the distance of the goal target in meters
   *
   *  @return
   *  Range: 0-30 meters
   */
  public double getGoalTargetDistance() {
    return goal_camera.getEntry("tz").getDouble(0);
  }

  /** Returns a boolean value for the intake target state
   *
   *  @return
   *  true: Intake Camera has a target.
   *  false: Intake Camera does not have a target.
   */
  public int getIntakeTargetsValid() {
    return (int)goal_camera.getEntry("tv").getNumber(0);
  }

  /** Returns the angle of the intake target in degrees.
   *
   *  @param targetIndex  value of the target index in descending order, with 0 being the most valid target.
   *
   *  @return +/- 20 degrees
   */
  public double getIntakeTargetAngle(int targetIndex) {
    double[] nullValue = {-99};
    var intakeAngles = goal_camera.getEntry("tx").getDoubleArray(nullValue);
     try {
       return intakeAngles[0] == -99 ? 0 : intakeAngles[targetIndex];
     } catch (Exception e) {
       System.out.println("Vision Subsystem Error: getIntakeTargetAngle() illegal array access");
       return 0;
     }
  }

  /** Returns the distance of the intake target in meters
   *
   *  @param targetIndex int value of the target index in descending order, with 0 being the most valid target.
   *
   *  @return
   *  Range: 0-30 meters
   */
  public double getIntakeTargetDistance(int targetIndex) {
    double[] nullValue = {-99};
    var intakeDistances = goal_camera.getEntry("tz").getDoubleArray(nullValue);
    try {
      return intakeDistances[0] == -99 ? 0 : intakeDistances[targetIndex];
    } catch (Exception e) {
      System.out.println("Vision Subsystem Error: getIntakeTargetDistance() illegal array access");
      return 0;
    }
  }

  /** Set the state of the Goal Camera LEDs. Does not do anything if the goal camera is an OAK device.
   *
   *  @param state
   *  true: Set LEDs On
   *  false: set LEDs Off
   */
  public void setGoalCameraLedState(boolean state) {
    int ledMode = 0;
    if(state) {
      switch (goal_camera_type) {
        case LIMELIGHT:
          ledMode = 3;
          break;
        case PHOTONVISION:
          ledMode = 1;
          break;
        case OAK_D:
        default:
          ledMode = 0;
          break;
      }
    } else {
      switch (goal_camera_type) {
        case LIMELIGHT:
          ledMode = 1;
          break;
        case PHOTONVISION:
        case OAK_D:
        default:
          ledMode = 0;
          break;
      }
    }

    goal_camera.getEntry("ledMode").setNumber(ledMode);
  }

  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("Has Goal Target", getGoalValidTarget());
    SmartDashboard.putNumber("Goal Angle", getGoalTargetXAngle());
    SmartDashboard.putNumber("Goal Distance", getGoalTargetDistance());

    SmartDashboard.putBoolean("Has Intake Target", getIntakeTargetsValid() > 0);
    SmartDashboard.putNumber("Intake Angle", getIntakeTargetAngle(0));
    SmartDashboard.putNumber("Intake Distance", getIntakeTargetDistance(0));

    SmartDashboard.putString("Alliance", getAllianceColor().toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
