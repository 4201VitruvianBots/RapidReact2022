// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    
    private boolean intaking = false; //is robot intaking
    
    //Intake motor setup
    private TalonFX intakeMotor =  new TalonFX(Constants.intakeMotor);
    
    //Intake piston setup
    DoubleSolenoid intakePiston = new DoubleSolenoid(Constants.pcmOne, Constants.intakePistonForward, Constants.intakePistonReverse);

    public Intake() {
        //Motor configuration
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
     * updates intake data on to the dashboard
     */
    public void updateSmartDashboard(){
        SmartDashboardTab.putBoolean("Intake", "Intake State", getIntakingState());
        SmartDashboardTab.putBoolean("Intake", "Pistons", getIntakePistonExtendStatus());
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
