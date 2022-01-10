// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
  /** Creates a new Outtake. */

        private final TalonFX[] outtakeMotors = {
        new TalonFX(Constants.Outtake.flywheelMotorA),
        new TalonFX(Constants.Outtake.flywheelMotorB)
        };
    private final PowerDistribution m_pdp;
    private final DriveTrain m_driveTrain;
    private final Vision m_vision;
    public int kI_Zone = 400;
    public int kAllowableError = 50;
    public double rpmOutput;
    public double rpmTolerance = 50.0;
    private double setpoint;
    private boolean timerStart;
    private double timestamp;
    private boolean canShoot;
    private double idealRPM;
        public Outtake(PowerDistribution powerdistribution, Vision vision, DriveTrain driveTrain ) {
        // Setup shooter motors (Falcons)
        for(TalonFX outtakeMotor : outtakeMotors) {
        outtakeMotor.configFactoryDefault();
        outtakeMotor.setNeutralMode(NeutralMode.Coast);
        outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
        outtakeMotor.configVoltageCompSaturation(10);
        outtakeMotor.enableVoltageCompensation(true);
        }
        m_vision = vision;
        m_driveTrain = driveTrain;
        m_pdp = powerdistribution;
    }
    /**
     * 
     * @param output
     * sets the controlmode percentoutput of outtakemotor0
     */
        public void setPower(double output) {
    }
    /**
     * 
     * @param setpoint
     * set to setpoint
     */
    public void setRPM(double setpoint) {
   
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean canShoot() {
        return canShoot;
    } 

    /**
     * updateRPMSetpoint else setPower to 0
     */
    private void updateRPMSetpoint() {
        
    }
    /**
     * set to test RPM
     */
    public void setTestRPM() {
        
    }

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }
    /**
     * boolean
     * @param motorIndex
     * return the absolute value of closed loop error < 100
     */
    public void encoderAtSetpoint() {   
    }

    /**
     * double
     * @param motorIndex
     * returns falcon units to RPM with outtake velocity
     */
    public void getRPM(int motorIndex) {
        
    } 
    public void setIdealRPM() {
        setpoint = idealRPM;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
    }
