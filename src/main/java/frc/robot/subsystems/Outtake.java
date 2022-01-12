// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Conversions;

import static frc.robot.Constants.Outtake.*;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */

    private final TalonFX[] outtakeMotors = {
      new TalonFX(Constants.Outtake.flywheelMotorA),
      new TalonFX(Constants.Outtake.flywheelMotorB)
    };

    private final Vision m_vision;
    private final Timer timeout = new Timer();
    public double rpmOutput;
    private double flywheelSetpointRPM;
    private double turretSetpoint; 
    private int controlMode; 
    private boolean initialHome; 
    private boolean canShoot; 
    private double idealRPM;

    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
          LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_observer =
          new KalmanFilter<>(
                  Nat.N1(),
                  Nat.N1(),
                  m_flywheelPlant,
                  VecBuilder.fill(3.0), // How accurate we think our model is
                  VecBuilder.fill(0.01), // How accurate we think our encoder
                  // data is
                  0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
            new LinearQuadraticRegulator<>(
                    m_flywheelPlant,
                    VecBuilder.fill(Conversions.RpmToRadPerSec(rpmTolerance)), // Velocity error tolerance
                    VecBuilder.fill(12.0), // Control effort (voltage) tolerance
                    0.020);

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop =
            new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    public Outtake(Vision vision) {
      // Setup shooter motors (Falcons)
      for(TalonFX outtakeMotor : outtakeMotors) {
          outtakeMotor.configFactoryDefault();
          outtakeMotor.setNeutralMode(NeutralMode.Coast);
          outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
          outtakeMotor.configVoltageCompSaturation(12);
          outtakeMotor.enableVoltageCompensation(true);
      }
      outtakeMotors[0].setInverted(true);
      outtakeMotors[1].follow(outtakeMotors[0], FollowerType.PercentOutput);

      m_vision = vision;
    }
    /**
     * 
     * @param output
     * sets the controlmode percentoutput of outtakemotor0
     */
    public void setPower(double output) {
      outtakeMotors[0].set(TalonFXControlMode.PercentOutput, output);
    }
    /**
     * 
     * @param setpoint
     * set to setpoint
     */
    public void setRPM(double flywheelSetpoint) {
   
    }

    /**
     *
     * @param setpoint
     * set to setpoint
     */
    public double getSetpointRPM() {
        return flywheelSetpointRPM;
    }

    public boolean canShoot() {
        return canShoot;
    } 

    /** flywheelSetpoint
     *  if setpoint else setPower to 0
     */
    private void updateRPMSetpoint() {
       if(getSetpointRPM() > 0) {
         m_loop.setNextR(VecBuilder.fill(Conversions.RpmToRadPerSec(flywheelSetpointRPM)));

         m_loop.correct(VecBuilder.fill(Conversions.RpmToRadPerSec(getRPM(0))));

         m_loop.predict(0.020);

         double nextVoltage = m_loop.getU(0);

         setPower(nextVoltage / 12.0);
       }
    }

    /**
     * set to test RPM
     */
    public void setTestRPM() {}

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }
    /**
     * boolean
     * @param motorIndex
     * @return the absolute value of closed loop error < 100
     */
    public void encoderAtSetpoint() {   
    }

    /**
     * double
     * @param motorIndex
     * @return falcon units to RPM with outtake velocity
     */
    public double getRPM(int motorIndex) {
        return outtakeMotors[motorIndex].getSelectedSensorVelocity() * (600.0 / encoderUnitsPerRotation);
    } 
    public void setIdealRPM() {
        flywheelSetpointRPM = idealRPM;
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
    public void getTurretAngle() {}

    /**
     * double
     * @return turret angle - drivetrain angle
     */
    public void getFieldRelativeAngle() {}

    /**
     * double
     * @return max angle
     */
    public void getMaxAngle() {}

    /** 
     * 
     * @return minimum angle
     * 
    */
    public void getMinAngle() {}

    /**
     * boolean
     * @return ! turrethomesensor.get
     */
    public void getTurretHome() {}

    public boolean getInitialHome() { 
      return initialHome;
    }
  
    /**
     * 
     * @param output
     * sets the percentoutput for the turretmotor
     */
    public void setPercentOutput(double output) {}

    /**
     * TurretSetpoint specifically 
     * sets the setpoint to turret angle
     */
    public void stopTurret() {}  

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
      updateRPMSetpoint();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
  }
