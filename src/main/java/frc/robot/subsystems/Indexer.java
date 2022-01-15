    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot.subsystems;

    import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

    public class Indexer extends SubsystemBase {
        private final double kI_Zone = 1;
        private final double maxVel = 1.1e4;
        private final double maxAccel = 1e6;
        private final double gearRatio = 1.0 / 27.0;

        
        // Setup indexer motor controller (SparkMax)
        CANSparkMax master = new CANSparkMax(Constants.Indexer.indexerMotor, MotorType.kBrushless);
        CANEncoder encoder = master.getEncoder();
        CANPIDController pidController = master.getPIDController();

        // PID terms/other constants
        private double kF = 0.0001;
        private double kP = 0.000001;
        private double kI = 80;
        private double kD = 0.0001;

        // Indexer sensors setup
        DigitalInput intakeSensor = new DigitalInput(Constants.Intake.intakeSensor);
        DigitalInput indexerTopSensor = new DigitalInput(Constants.Indexer.indexerTopSensor);
        DigitalInput indexerBottomSensor = new DigitalInput(Constants.Indexer.indexerBottomSensor);

        /** Creates a new Indexer. */
        public Indexer() {
            // Motor and PID controller setup
            master.restoreFactoryDefaults();
            master.setInverted(true);
            master.setSmartCurrentLimit(10, 10);

            master.setIdleMode(IdleMode.kCoast);

            pidController.setFF(kF);
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
            pidController.setSmartMotionMaxVelocity(maxVel, 0); // Formerly 1.1e4
            pidController.setSmartMotionMaxAccel(maxAccel, 0); // Formerly 1e6
            pidController.setSmartMotionAllowedClosedLoopError(1, 0);
            pidController.setIZone(kI_Zone);

            initShuffleboard();
        }

        /**
         * Sets control mode to either 1 or 0
         */
        public void toggleControlMode(){
            if(controlMode == 0)
            controlMode = 1;
          else
            controlMode = 0;
        }

        /**
         * @return Returns the control mode value 
         */
        public int getControlMode(){
            return controlMode;
        }

        /**
         * sets the power for the kicker motor 
         * @param output value for the power of the kicker motor
         */
        public void setKickerOutput(double output){
            kicker.set(ControlMode.PercentOutput, output);
        }

        /**
         * Sets the power for the indexer motor
         * @param output value for the power of the indexer motor
         */
        public void setIndexerOutput(double output){
            master.set(output);
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
         * 
         * @return
         */
        public boolean getIndexerTopSensor() {
            return ! indexerTopSensor.get();
        }

        /**
         * Sets the RPM of the indexer
         * @param rpm value for the rpm
         */
        public void setRPM(double rpm) {
            double setpoint = rpm / gearRatio;
            SmartDashboard.putNumber("Indexer Setpoint", setpoint);
            pidController.setReference(setpoint, ControlType.kSmartVelocity);
        }

        /**
         * updates the SmartDashboard with Indexer values
         */
        private void updateSmartDashboard(){
            SmartDashboardTab.putNumber("Indexer", "Carousel RPM", this.getRPM());
            SmartDashboard.putNumber("kF", kF);
            SmartDashboard.putNumber("kP", kP);
            SmartDashboard.putNumber("kI", kI);
            SmartDashboard.putNumber("kD", kD);
        }

        /**
         * updates PID values onto the controllers and smartdashboard
         */
        private void updatePIDValues() {
            // Allow PID values to be set through SmartDashboard
            kF = SmartDashboard.getNumber("kF", 0);
            kP = SmartDashboard.getNumber("kP", 0);
            kI = SmartDashboard.getNumber("kI", 0);
            kD = SmartDashboard.getNumber("kD", 0);
            pidController.setFF(kF);
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
        }
        @Override
        public void periodic() {
            // This method will be called once per scheduler run
            updateSmartDashboard();
            updatePIDValues();
        }

        @Override
        public void simulationPeriodic() {
            // This method will be called once per scheduler run during simulation
        }
    }
