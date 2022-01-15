// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  private final int encoderUnitsPerRotation = 4096;
    private final DriveTrain m_driveTrain;
    private final Timer timeout = new Timer();
    private final CANCoder encoder = new CANCoder(Constants.Turret.turretEncoder);
    private final VictorSPX turretMotor = new VictorSPX(Constants.Turret.turretMotor);
    private final DigitalInput turretHomeSensor = new DigitalInput(Constants.Turret.turretHomeSensor);
    double minAngle = - 90;  // -135;
    double maxAngle = 90;   // 195;
    double gearRatio = 18.0 / 120.0;
    private double setpoint = 0; //angle
    private int controlMode = 1;
    private boolean initialHome;
    private boolean turretHomeSensorLatch = false;
  /** Creates a new Turret. */
  public Turret(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    encoder.configFactoryDefault();
    encoder.setPositionToAbsolute();
    encoder.configSensorDirection(true);
  }
  public void resetEncoder() {
    turretMotor.setSelectedSensorPosition(0);
    //encoder.setPosition(0);
  }

  public int getControlMode() {
    return controlMode;
  }

  public void setControlMode(int mode) {
    controlMode = mode;
  }

  /**
   * double
   * returns encoder units of turret into degrees
   */
  public double getTurretAngle() {
    return encoderUnitsToDegrees(turretMotor.getSelectedSensorPosition());
  }

  /**
   * double
   * @return turret angle - drivetrain angle
   */
  public double getFieldRelativeAngle() {
    return getTurretAngle(); //- m_driveTrain.getAngle()
  }

  /**
   * double
   * @return max angle
   */
  public double getMaxAngle() {
    return maxAngle;
  }

  /** 
   * 
   * @return minimum angle
   * 
  */
  public double getMinAngle() {
    return minAngle;
  }

  /**
   * boolean
   * @return ! turrethomesensor.get
   */
  public boolean getTurretHome() {
    return ! turretHomeSensor.get();
  }

  public boolean getInitialHome() { 
    return initialHome;
  }

  /**
   * 
   * @param output
   * sets the percentoutput for the turretmotor
   */
  public void setPercentOutput(double output) {
    turretMotor.set(ControlMode.PercentOutput, output);
  }

  public void setRobotCentricSetpoint(double setpoint){
    this.setpoint = setpoint;
  }
  /**
   * TurretSetpoint specifically 
   * sets the setpoint to turret angle
   */
  public void stopTurret() {
    setpoint = getTurretAngle();
  }
  public int degreestoEncoderunits(double degrees) {
    return (int) (degrees * (1.0 / gearRatio) * (encoderUnitsPerRotation / 360.0));

  }
public double encoderUnitsToDegrees(double encoderUnits) {
  return encoderUnits * gearRatio * (360.0 / encoderUnitsPerRotation);

}
  public boolean getTurretLatch() {
    return turretHomeSensorLatch;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
