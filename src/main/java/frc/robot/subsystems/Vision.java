// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.Constants.Vision.CAMERA_TYPE;
import frc.robot.Constants.Vision.INTAKE_TRACKING_TYPE;
import frc.robot.simulation.FieldSim;
import frc.robot.utils.VisionData;
import javax.swing.text.Position;

public class Vision extends SubsystemBase {

  private final Controls m_controls;
  private final DriveTrain m_drivetrain;
  private final Turret m_turret;
  private final FieldSim m_fieldSim;

  private final NetworkTable goal_camera;
  private final NetworkTable intake_camera;
  private final NetworkTable limelight;

  private CAMERA_TYPE goal_camera_type = CAMERA_TYPE.OAK;
  private int red_offset, blue_offset;

  private INTAKE_TRACKING_TYPE intake_tracking_type = INTAKE_TRACKING_TYPE.CARGO;

  private DigitalOutput rsl = new DigitalOutput(5);

  private VisionData[] dataBuffer = new VisionData[100];
  private int bufferIdx = 0;

  private boolean enablePoseEstimation = false;

  private double distanceFromLimelightToGoalMeters;
  private Pose2d hubPose;
  private Rotation2d rotation;
  private double theta, x, y;
  private Translation2d robotPose;
  private int bufferLength;
  private VisionData data, item;
  private double lastOakPoseTimestamp;
  private double angle, robotVelocity, angularVelocity, tangentalVelocity, ff;

  /** Creates a new Vision Subsystem. */
  public Vision(Controls controls, DriveTrain driveTrain, Turret turret, FieldSim fieldSim) {
    m_controls = controls;
    m_drivetrain = driveTrain;
    m_turret = turret;
    m_fieldSim = fieldSim;

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
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    intake_camera = NetworkTableInstance.getDefault().getTable("OAK-1_Intake");

    PortForwarder.add(5800, Constants.Vision.LIMELIGHT_IP, 5800);
    PortForwarder.add(5801, Constants.Vision.LIMELIGHT_IP, 5801);
    PortForwarder.add(5802, Constants.Vision.LIMELIGHT_IP, 5802);
    PortForwarder.add(5803, Constants.Vision.VISION_SERVER_IP, 5802);
  }

  /**
   * Given a camera, return a boolean value based on if it sees a target or not.
   *
   * @return true: Camera has a target. false: Camera does not have a target
   */
  public boolean getValidTarget(CAMERA_POSITION position) {
    switch (position) {
      case GOAL:
        return goal_camera.getEntry("tv").getDouble(0) == 1;
      case INTAKE:
        return intake_camera.getEntry("tv").getDouble(0) > 0;
      case LIMELIGHT:
        return limelight.getEntry("tv").getDouble(0) == 1;
      default:
        return false;
    }
  }

  /**
   * Given a camera, return a boolean value if it's detection is good (Check for additional
   * parameters to validate a detection.
   *
   * @return true: Goal target can be used for distance calculations. false: Goal target cannot be
   *     used for distance calculations.
   */
  public boolean getGoodTarget(CAMERA_POSITION position) {
    switch (position) {
      case GOAL:
        return goal_camera.getEntry("tg").getDouble(0) == 1;
      case INTAKE:
      default:
        return false;
    }
  }

  /**
   * Returns the vertical angle of a camera's target in degrees. For the intake, can specify index
   * for multiple targets.
   *
   * @return Vertical angle (+/- 35 degrees)
   */
  public double getTargetXAngle(CAMERA_POSITION position) {
    return getTargetXAngle(position, 0);
  }

  public double getTargetXAngle(CAMERA_POSITION position, int index) {
    switch (position) {
      case GOAL:
        return goal_camera.getEntry("tx").getDouble(0); // TODO MEMORY LEAK
      case INTAKE:
        if (getValidTarget(position)) {
          double[] nullValue = {-99};
          var intakeAngles = intake_camera.getEntry("tx").getDoubleArray(nullValue);
          try {
            return intakeAngles[0] == -99 ? 0 : intakeAngles[index];
          } catch (Exception e) {
            System.out.println("Vision Subsystem Error: getTargetXAngle() illegal array access");
            return 0;
          }
        } else return 0;
      case LIMELIGHT:
        return limelight.getEntry("tx").getDouble(0);
      default:
        return 0;
    }
  }

  /**
   * Returns the vertical angle of a camera's target as a Rotation2d. For the intake, can specify
   * index for multiple targets.
   *
   * @return Vertical angle (+/- 35 degrees)
   */
  public Rotation2d getTargetXRotation2d(CAMERA_POSITION position) {
    return getTargetXRotation2d(position, 0);
  }

  public Rotation2d getTargetXRotation2d(CAMERA_POSITION position, int index) {
    return new Rotation2d(Units.degreesToRadians(getTargetXAngle(position, index)));
  }

  /**
   * Returns the horizontal angle of the goal target in degrees. For the intake, can specify index
   * for multiple targets.
   *
   * @return Horizontal angle (+/- 27 degrees)
   */
  public double getTargetYAngle(CAMERA_POSITION position) {
    return getTargetYAngle(position, 0);
  }

  public double getTargetYAngle(CAMERA_POSITION position, int index) {
    switch (position) {
      case GOAL:
        return goal_camera.getEntry("ty").getDouble(0);

      case LIMELIGHT:
        return limelight.getEntry("ty").getDouble(0);

      case INTAKE:
        if (getValidTarget(position)) {
          double[] nullValue = {-99};
          var intakeAngles = intake_camera.getEntry("ty").getDoubleArray(nullValue);
          try {
            return intakeAngles[0] == -99 ? 0 : intakeAngles[index];
          } catch (Exception e) {
            System.out.println("Vision Subsystem Error: getTargetYAngle() illegal array access");
            return 0;
          }
        }
      default:
        return 0;
    }
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
  public double getGoalTargetHorizontalDistance(CAMERA_POSITION Position) {
    switch (Position) {
      case LIMELIGHT:
        if (getValidTarget(CAMERA_POSITION.LIMELIGHT)) {
          distanceFromLimelightToGoalMeters =
              (Constants.Vision.UPPER_HUB_HEIGHT_METERS
                      - Constants.Vision.LIMELIGHT_MOUNTING_HEIGHT_METERS)
                  / Math.tan(
                      Units.degreesToRadians(
                          Constants.Vision.LIMELIGHT_MOUNTING_ANGLE_DEGREES
                              + getTargetYAngle(CAMERA_POSITION.LIMELIGHT)));
          return distanceFromLimelightToGoalMeters;
        }
      case GOAL:
        return Math.cos(
                Units.degreesToRadians(
                    Constants.Vision.GOAL_CAMERA_MOUNTING_ANGLE_DEGREES
                        + getTargetYAngle(CAMERA_POSITION.GOAL, 0)))
            * getGoalTargetDirectDistance();
      default:
        return 0;
    }
  }
  public double getCargoTargetDirectDistance() {
    return getCargoTargetDirectDistance(0);
  }

  public double getCargoTargetDirectDistance(int index) {
    if (getValidTarget(CAMERA_POSITION.INTAKE)) {
      double[] nullValue = {-99};
      var intakeAngles = intake_camera.getEntry("tz").getDoubleArray(nullValue);
      try {
        return intakeAngles[0] == -99 ? 0 : intakeAngles[index];
      } catch (Exception e) {
        System.out.println("Vision Subsystem Error: getTargetYAngle() illegal array access");
        return 0;
      }
    } else
      return -1;
  }

  public double getCargoHorizontalDistance() {
    return getCargoHorizontalDistance(0);
  }

  public double getCargoHorizontalDistance(int index) {
    return Math.cos(
            Units.degreesToRadians(
                    Constants.Vision.INTAKE_CAMERA_MOUNTING_ANGLE_DEGREES
                            + getTargetYAngle(CAMERA_POSITION.INTAKE, index)))
            * getTargetYAngle(CAMERA_POSITION.INTAKE, index);
  }

  /**
   * Get the pose of the hub. We just rotate it 180 degrees if we're blue for pose estimation.
   *
   * @return Hub Pose in meters
   */
  public Pose2d getHubPose() {
    hubPose = Constants.Vision.HUB_POSE;
    rotation = new Rotation2d();
    //        new Rotation2d(m_controls.getAllianceColorBoolean() ? Units.degreesToRadians(180) :
    // 0);
    return new Pose2d(hubPose.getTranslation(), rotation);
  }

  /**
   * Calculate the robot's pose relative to the hub based on vision tracking.
   *
   * @return Robot Pose in meters
   */
  public Pose2d getPoseFromHub(CAMERA_POSITION position) {
    theta =
        m_drivetrain
            .getHeadingRotation2d()
            .plus(m_turret.getTurretRotation2d())
            .minus(getTargetXRotation2d(position))
            .getRadians();

    x = (getGoalTargetHorizontalDistance(position) * Math.cos(theta));
    y = (getGoalTargetHorizontalDistance(position) * Math.sin(theta));

    robotPose =
        new Translation2d(x, y)
            .rotateBy(getHubPose().getRotation())
            .plus(getHubPose().getTranslation());

    return new Pose2d(robotPose, m_drivetrain.getHeadingRotation2d());
  }

  /**
   * Get the timestamp of the detection results.
   *
   * @return Robot Pose in meters
   */
  public double getDetectionTimestamp() {
    return goal_camera.getEntry("timestamp").getDouble(0);
  }

  public double getLimelightLatency() {
    return limelight.getEntry("tl").getDouble(0);
  }

  /**
   * Set the state of the Goal Camera LEDs. Does not do anything if the goal camera is an OAK
   * device.
   *
   * @param state true: Set LEDs On false: set LEDs Off
   */
  public void setLimelightLEDState(boolean state) {

    limelight.getEntry("ledMode").setNumber(state ? 3 : 1);
  }

  public void setLimelightPipeline(int pipeline) {
    limelight.getEntry("pipeline").setNumber(pipeline);
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

  /** Get VisionData given a tiemstamp. Will return first result if no valid results are found */
  public VisionData getTimestampedData(double timestamp) {
    bufferLength = bufferIdx > dataBuffer.length ? dataBuffer.length : bufferIdx;
    for (int i = bufferLength; i > -1; i--) {
      data = dataBuffer[i];
      if (data.timestamp < timestamp) {
        return data;
      }
    }
    return dataBuffer[0];
  }

  /** Update our data buffer of saved VisionData for latency compensation */
  private void updateDataBuffer() {
    item =
        new VisionData(
            getDetectionTimestamp(),
            getTargetXAngle(CAMERA_POSITION.GOAL),
            getTargetYAngle(CAMERA_POSITION.GOAL),
            m_drivetrain.getRobotPoseMeters());

    if (bufferIdx > dataBuffer.length - 1) {
      System.arraycopy(dataBuffer, 0, dataBuffer, 1, dataBuffer.length - 1);
      bufferIdx = dataBuffer.length - 1;
    }
    dataBuffer[bufferIdx] = item;

    bufferIdx++;
  }

  public void setVisionPoseEstimation(boolean enabled) {
    enablePoseEstimation = false;
  }

  /** Update the robot pose based on vision data if a valid vision target is found. */
  private void updateVisionPose() {
    if (enablePoseEstimation) {
      if (getValidTarget(CAMERA_POSITION.GOAL)) {
        m_drivetrain
            .getOdometry()
            .addVisionMeasurement(
                getPoseFromHub(CAMERA_POSITION.GOAL),
                Timer.getFPGATimestamp() - 0.267); // Vision camera has ~ 267 ms of latency
        lastOakPoseTimestamp = Timer.getFPGATimestamp();
      } else if (getValidTarget(CAMERA_POSITION.LIMELIGHT)
          && (Timer.getFPGATimestamp() - lastOakPoseTimestamp) > 0.5) {
        m_drivetrain
            .getOdometry()
            .addVisionMeasurement(
                getPoseFromHub(CAMERA_POSITION.LIMELIGHT),
                Timer.getFPGATimestamp() - (getLimelightLatency() + 0.011));
      }
    }
  }

  /** Give the turret a feedforward value if the robot is moving. Based on 254's 2019 code */
  private void updateTurretArbitraryFF() {
    angle =
        Math.sin(
            m_turret.getTurretRotation2d().minus(m_drivetrain.getHeadingRotation2d()).getRadians());
    robotVelocity =
        (m_drivetrain.getSpeedsMetersPerSecond().leftMetersPerSecond
                + m_drivetrain.getSpeedsMetersPerSecond().rightMetersPerSecond)
            / 2.0;
    angularVelocity = 0;
    if (getValidTarget(CAMERA_POSITION.LIMELIGHT)) {
      angularVelocity =
          angle * robotVelocity / getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT);
    } else if (getValidTarget(CAMERA_POSITION.GOAL))
      angularVelocity =
          angle * robotVelocity / getGoalTargetHorizontalDistance(CAMERA_POSITION.GOAL);

    tangentalVelocity = Units.degreesToRadians(m_drivetrain.getHeadingRateDegrees());
    ff = (angularVelocity + tangentalVelocity) * 0.006;

    //    SmartDashboardTab.putNumber("Vision", "Turret FF", ff);

    m_turret.setArbitraryFF(ff);
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    // SmartDashboard.putBoolean("Has Goal Target", getValidTarget(CAMERA_POSITION.GOAL));
    // SmartDashboard.putNumber("Goal Angle", getTargetXAngle(CAMERA_POSITION.GOAL));
    SmartDashboard.putNumber(
        "Goal Horizontal Distance", getGoalTargetHorizontalDistance(CAMERA_POSITION.GOAL));
    SmartDashboard.putNumber(
        "Limelight Target Distance", getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT));

    SmartDashboard.putBoolean("Has Intake Target", getValidTarget(CAMERA_POSITION.INTAKE));
    SmartDashboard.putNumber("Intake Angle", getTargetXAngle(CAMERA_POSITION.INTAKE, 0));

    // SmartDashboardTab.putNumber(
    //     "Vision", "Hub Horizontal Distance", getGoalTargetHorizontalDistance());
    // SmartDashboardTab.putNumber("Vision", "Hub X Angle", getTargetXAngle(CAMERA_POSITION.GOAL));
    // SmartDashboardTab.putNumber("Vision", "Hub Y Angle", getTargetYAngle(CAMERA_POSITION.GOAL));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    //    updateDataQueue();
    updateVisionPose();
    // updateTurretArbitraryFF();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
