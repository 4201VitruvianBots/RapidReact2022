// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
    public Intake() {

    }

    /**
    * @return Gets a boolean for the intake's actuation
    */
    public boolean getIntakeState() {
        return false;
    }

    /**
    * Sets a boolean for the intake's actuation
    */
    public void setIntakeState(){
      
    }

    /**
    * @return A boolean value based on the intake's piston status (up or down)
    */
    public boolean getIntakePistonExtendStatus(){
        return false;
    }

    /**
     * Sets intake piston's states to forward and backward
     */
    public void setIntakePiston(){
    
    }

    /**
     * sets the amount of power going to the intake
     */
    public void setIntakePercentOutput(){

    }

    /**
     * updates intake data on to the dashboard
     */
    public void updateSmartDashboard(){
    
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
