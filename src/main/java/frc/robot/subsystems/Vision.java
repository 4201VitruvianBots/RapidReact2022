// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.VisionData;

public class Vision extends SubsystemBase {

  private final Controls m_controls;
  private final DriveTrain m_drivetrain;
  private final Turret m_turret;
  private final NetworkTable goal_camera;
  private final NetworkTable intake_camera;

  private CAMERA_TYPE goal_camera_type = CAMERA_TYPE.OAK;
  private int red_offset, blue_offset;

  private INTAKE_TRACKING_TYPE intake_tracking_type = INTAKE_TRACKING_TYPE.CARGO;

  private DigitalOutput rsl = new DigitalOutput(5);

  private VisionData[] dataBuffer = new VisionData[100];
  private int bufferIdx = 0;

  /** Creates a new Vision Subsystem. */
  public Vision(Controls controls, DriveTrain driveTrain, Turret turret) {
    m_controls = controls;
    m_drivetrain = driveTrain;
    m_turret = turret;

    switch (goal_camera_type) {
      case PHOTONVISION:
        goal_camera = NetworkTableInstance.getDefault().getTable("photonvision");
        break;
      case LIMELIGHT:
        goal_camera = NetworkTableInstance.getDefault().getTable("limelight");
        break;
      case OAK:
      default:
        goal_camera = NetworkTableInstance.getDefault().getTable("OAK-D_Goal");
        break;
    }
    intake_camera = NetworkTableInstance.getDefault().getTable("OAK-1_Intake");

    PortForwarder.add(5802, VISION_SERVER_IP, 5802);
  }
  /**
   * Returns a boolean value based on checks to determine if the robot can shoot.
   *
   * @return true: All shooting parameters passed, so the robot can shoot false: One or more
   *     parameters failed, so the robot shouldn't shoot
   */
  public boolean canShoot() {
    return MIN_SHOOTING_DISTANCE < getGoalTargetHorizontalDistance()
        && getGoalTargetHorizontalDistance() < MAX_SHOOTING_DISTANCE
        && getGoalValidTarget();
  }

  /**
   * Returns a boolean value to determine if the goal camera has a visible target.
   *
   * @return true: Goal Camera has a target. false: Goal Camera does not have a target.
   */
  public boolean getGoalValidTarget() {
    return goal_camera.getEntry("tv").getDouble(0) == 1;
  }

  /**
   * Returns a boolean value to determine if the goal target can be used for distance calculations.
   *
   * @return true: Goal target can be used for distance calculations. false: Goal target cannot be
   *     used for distance calculations.
   */
  public boolean getGoalGoodTarget() {
    return goal_camera.getEntry("tg").getDouble(0) == 1;
  }

  /**
   * Returns the vertical angle of the goal target in degrees.
   *
   * @return Vertical angle (+/- 20 degrees)
   */
  public double getGoalTargetXAngle() {
    return -goal_camera.getEntry("tx").getDouble(0);
  }

  public Rotation2d getGoalTargetXRotation2d() {
    return new Rotation2d(Units.degreesToRadians(getGoalTargetXAngle())).unaryMinus();
    //    return new Rotation2d(Units.degreesToRadians(10));
  }

  /**
   * Returns the horizontal angle of the goal target in degrees.
   *
   * @return Horizontal angle (+/- 20 degrees)
   */
  public double getGoalTargetYAngle() {
    return goal_camera.getEntry("ty").getDouble(0);
  }

  /**
   * Returns the distance of the goal target in meters
   *
   * @return Direct Distance to goal target (0-30 meters)
   */
  public double getGoalTargetDirectDistance() {
    return goal_camera.getEntry("tz").getDouble(0);
  }

  /**
   * Calculates the horizontal distance to the goal target using the Upper Hub.
   *
   * @return Horizontal Distance to goal target (0-30 meters)
   */
  public double getGoalTargetHorizontalDistance() {
    return Math.cos(
            Units.degreesToRadians(GOAL_CAMERA_MOUNTING_ANGLE_DEGREES + getGoalTargetYAngle()))
        * getGoalTargetDirectDistance();
  }

  public Pose2d getPoseFromHub() {
    double theta =
        m_drivetrain
            .getHeadingRotation2d()
            .plus(m_turret.getTurretRotation2d())
            .minus(getGoalTargetXRotation2d())
            .getRadians();

    //    Pose2d hubPose = HUB_POSE ? m_controls.getAllianceColor()

    double x = (getGoalTargetHorizontalDistance() * Math.cos(theta)) + HUB_POSE.getX();
    double y = (getGoalTargetHorizontalDistance() * Math.sin(theta)) + HUB_POSE.getY();

    return new Pose2d(x, y, m_drivetrain.getHeadingRotation2d());
  }

  public double getDetectionTimestamp() {
    return goal_camera.getEntry("timestamp").getDouble(0);
  }

  /**
   * Returns a boolean value for the intake target state
   *
   * @return true: Intake Camera has a target. false: Intake Camera does not have a target.
   */
  public int getIntakeTargetsValid() {
    return (int) intake_camera.getEntry("tv").getDouble(0);
  }

  /**
   * Returns the angle of the intake target in degrees.
   *
   * @param targetIndex value of the target index in descending order, with 0 being the most valid
   *     target.
   * @return +/- 20 degrees
   */
  public double getIntakeTargetAngle(int targetIndex) {
    if (getIntakeTargetsValid() == 1) {
      double[] nullValue = {-99};
      var intakeAngles = intake_camera.getEntry("ta").getDoubleArray(nullValue);
      try {
        return intakeAngles[0] == -99 ? 0 : intakeAngles[targetIndex];
      } catch (Exception e) {
        System.out.println("Vision Subsystem Error: getIntakeTargetAngle() illegal array access");
        return 0;
      }
    } else return 0;
  }

  /**
   * Set the state of the Goal Camera LEDs. Does not do anything if the goal camera is an OAK
   * device.
   *
   * @param state true: Set LEDs On false: set LEDs Off
   */
  public void setGoalCameraLedState(boolean state) {
    int ledMode = 0;
    if (state) {
      switch (goal_camera_type) {
        case LIMELIGHT:
          ledMode = 3;
          break;
        case PHOTONVISION:
          ledMode = 1;
          break;
        case OAK:
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
        case OAK:
        default:
          ledMode = 0;
          break;
      }
    }

    goal_camera.getEntry("ledMode").setNumber(ledMode);
  }

  /** Returns the total count of red cargo detected by the intake camera. */
  public double getRedCount() {
    return intake_camera.getEntry("red_count").getDouble(0);
  }

  /** Returns the total count of blue cargo detected by the intake camera. */
  public double getBlueCount() {
    return intake_camera.getEntry("blue_count").getDouble(0);
  }

  /** Updates the offset value for the red counter display on the intake video feed. */
  public void updateRedOffset() {
    red_offset += getRedCount();
    intake_camera.getEntry("red_counter_offset").setDouble(red_offset);
  }

  /** Updates the offset value for the blue counter display on the intake video feed. */
  public void updateBlueOffset() {
    blue_offset += getBlueCount();
    intake_camera.getEntry("blue_counter_offset").setDouble(blue_offset);
  }

  /** Sets the category of objects the intake should track. 0: Cargo 1: Launchpads */
  public void setIntakeTrackingType(INTAKE_TRACKING_TYPE type) {
    SmartDashboardTab.putNumber("Vision", "intake_tracking_type", type.ordinal());
  }
  /** Sets the category of objects the intake should track. 0: Cargo 1: Launchpads */
  public void setIntakeTargetLock(boolean state) {
    SmartDashboardTab.putBoolean("Vision", "intake_target_lock", state);
  }

  public VisionData getTimestampedData(double timestamp) {
    int bufferLength = bufferIdx > dataBuffer.length ? dataBuffer.length : bufferIdx;
    for (int i = bufferLength; i > -1; i--) {
      VisionData data = dataBuffer[i];
      if (data.timestamp < timestamp) {
        return data;
      }
    }
    return dataBuffer[0];
  }

  private void updateDataQueue() {
    VisionData item =
        new VisionData(
            getDetectionTimestamp(),
            getGoalTargetXAngle(),
            getGoalTargetYAngle(),
            m_drivetrain.getRobotPoseMeters());

    if (bufferIdx > dataBuffer.length - 1) {
      System.arraycopy(dataBuffer, 0, dataBuffer, 1, dataBuffer.length - 1);
      bufferIdx = dataBuffer.length - 1;
    }
    dataBuffer[bufferIdx] = item;

    bufferIdx++;
  }

  private void updateVisionPose() {
    if (getGoalValidTarget()) {
      m_drivetrain
          .getOdometry()
          .addVisionMeasurement(
              getPoseFromHub(),
              Timer.getFPGATimestamp() - 0.267); // Vision camera has ~ 2.67 ms of latency
    }
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("Has Goal Target", getGoalValidTarget());
    SmartDashboard.putNumber("Goal Angle", getGoalTargetXAngle());
    SmartDashboard.putNumber("Goal Direct Distance", getGoalTargetDirectDistance());
    SmartDashboard.putNumber("Goal Horizontal Distance", getGoalTargetHorizontalDistance());

    SmartDashboard.putBoolean("Has Intake Target", getIntakeTargetsValid() > 0);
    SmartDashboard.putNumber("Intake Angle", getIntakeTargetAngle(0));

    SmartDashboardTab.putNumber(
        "Vision", "Hub Horizontal Distance", getGoalTargetHorizontalDistance());
    SmartDashboardTab.putNumber("Vision", "Hub X Angle", getGoalTargetXAngle());
    SmartDashboardTab.putNumber("Vision", "Hub Y Angle", getGoalTargetYAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    //    updateDataQueue();
    updateVisionPose();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
