// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Outtake. */

      private final TalonFX[] shooterMotors = {
      new TalonFX(Constants.Shooter.flywheelMotorA),
      new TalonFX(Constants.Shooter.flywheelMotorB)
      };

    private final Vision m_vision;
    private final int encoderUnitsPerRotation = 4096;
    private final Timer timeout = new Timer();
    public double rpmOutput;
    public double rpmTolerance = 50.0;
    private double flywheelSetpointRPM;
    private double turretSetpoint; 
    private int controlMode; 
    private boolean initialHome; 
    private boolean canShoot; 
    private double idealRPM; 

    
    public Shooter(PowerDistribution powerdistribution, Vision vision, DriveTrain driveTrain ) {
    // Setup shooter motors (Falcons)
    for(TalonFX outtakeMotor : shooterMotors) {
        outtakeMotor.configFactoryDefault();
        outtakeMotor.setNeutralMode(NeutralMode.Coast);
        outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
        outtakeMotor.configVoltageCompSaturation(10);
        outtakeMotor.enableVoltageCompensation(true);
    }
        shooterMotors[0].setInverted(true);
        shooterMotors[1].follow(shooterMotors[0], FollowerType.PercentOutput);

        m_vision = vision;
    }
    /**
     * 
     * @param output
     * sets the controlmode percentoutput of outtakemotor0
     */
        public void setPower(double output) {
            shooterMotors[0].set(ControlMode.PercentOutput, output);
    }
    /**
     * 
     * @param setpoint
     * set to setpoint
     */
    public void setRPM(double flywheelSetpointRPM) {
    this.flywheelSetpointRPM = flywheelSetpointRPM;
    }

    public double getSetpoint() {
        return  flywheelSetpointRPM;
    }

    public boolean canShoot() {
        return canShoot;
    } 

    /** flywheelSetpointRPM
     *  if setpoint else setPower to 0
     */
    private void updateRPMSetpoint() {
        if(flywheelSetpointRPM >= 0)
        shooterMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(flywheelSetpointRPM));
    else
        setPower(0);   
    }
    /**
     * set to test RPM
     */
    public void setTestRPM() {
        shooterMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
    }

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }
    
    public double getRPM(int motorIndex) {
        return FalconUnitstoRPM(shooterMotors[motorIndex].getSelectedSensorVelocity());
    } 

    public double FalconUnitstoRPM(double SensorUnits){
        return(SensorUnits / 2048.0)*600.0;
    }

    public double RPMtoFalconUnits(double RPM){
        return(RPM / 600.0)*2048.0;
    }
    public void setIdealRPM() {
        flywheelSetpointRPM = idealRPM;
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
