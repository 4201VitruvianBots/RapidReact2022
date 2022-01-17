// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    
    private boolean intaking = false; //is robot intaking
    
    //Intake motor setup
    private TalonFX intakeMotor =  new TalonFX(Constants.Intake.intakeMotor);
    
    //Intake piston setup
    DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.Intake.pcmOne, PneumaticsModuleType.CTREPCM, Constants.Intake.intakePistonForward, Constants.Intake.intakePistonReverse);

    public Intake() {
        //Motor configuration
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(false);
    }

    /**
    * @return Gets a boolean for the intake's actuation
    */
    public boolean getIntakeState() {
        return intaking;
    }

    /**
    * Sets a boolean for the intake's actuation
    */
    public void setIntakeState(boolean state){
        intaking = state; 
    }

    /**
    * @return A boolean value based on the intake's piston status (up or down)
    */
    public boolean getIntakePistonExtendStatus(){
        return intakePiston.get() == DoubleSolenoid.Value.kForward ? true : false;
    }

    /**
     * Sets intake piston's states to forward and backward
     */
    public void setIntakePiston(boolean state){
        intakePiston.set(state ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    /**
     * sets the amount of power going to the intake
     */
    public void setIntakePercentOutput(double value){
        intakeMotor.set(ControlMode.PercentOutput, value);
    }

    /**
     * 
     * @param intaking is used in this method
     * @param intakePiston is used in this method
     * @return Returns the state of the Intake and the intakePiston
     */
    public boolean getIntakeStatus(boolean intaking, boolean intakePiston){
        return intaking && intakePiston;
    }

    /**
     * updates intake data on to the dashboard
     */
    public void updateSmartDashboard(){
        //SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakeState());
        //SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
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
