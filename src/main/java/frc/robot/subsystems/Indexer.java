    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot.subsystems;

    import edu.wpi.first.wpilibj2.command.SubsystemBase;

    public class Indexer extends SubsystemBase {
    /** Creates a new Indexer. */
        public Indexer() {

        }

        /**
         * Sets control mode to either 1 or 0
         */
        public void toggleControlMode(){

        }

        /**
         * @return Returns the control mode value 
         */
        public int getControlMode(){
            return 0;
        }

        /**
         * sets the power for the kicker motor 
         * @param output value for the power of the kicker motor
         */
        public void setKickerOutput(double output){

        }

        /**
         * Sets the power for the indexer motor
         * @param output value for the power of the indexer motor
         */
        public void setIndexerOutput(double output){

        }

        /**
         *  Senses if there is a ball in the indexer
         *  don't know if we need it
         * @return if there is a ball in the indexer
         */
        public boolean newBall(){
            return false;
        }

        /**
         * Sets the RPM of the indexer
         * @param rpm value for the rpm
         */
        public void setRPM(double rpm) {

        }

        /**
         * updates the SmartDashboard with Indexer values
         */
        private void updateSmartDashboard(){

        }

        /**
         * updates PID values onto the controllers and smartdashboard
         */
        private void updatePIDValue0() {

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
