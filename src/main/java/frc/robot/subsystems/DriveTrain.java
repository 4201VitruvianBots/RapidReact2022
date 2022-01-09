// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A differential drivetrain with two falcon motors on each side
 */
public class DriveTrain extends SubsystemBase {

    private final double gearRatio = 1.0 / 5.0;

    private final double kS = Constants.DriveTrain.ksVolts;
    private final double kV = Constants.DriveTrain.kvVoltSecondsPerMeter;
    private final double kA = Constants.DriveTrain.kaVoltSecondsSquaredPerMeter;

    public double kP = 2; //3.6294;
    public double kI = 0;
    public double kD = 0;
    public int controlMode = 0;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(21.5));
    DifferentialDriveOdometry odometry;
    DifferentialDrivePoseEstimator m_poseEstimator;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    PIDController leftPIDController = new PIDController(kP, kI, kD);
    PIDController rightPIDController = new PIDController(kP, kI, kD);

    private final TalonFX[] driveMotors = {
            new TalonFX(Constants.DriveTrain.leftFrontDriveMotor),
            new TalonFX(Constants.DriveTrain.leftRearDriveMotor),
            new TalonFX(Constants.DriveTrain.rightFrontDriveMotor),
            new TalonFX(Constants.DriveTrain.rightRearDriveMotor)
    };
    double m_leftOutput, m_rightOutput;

    private final boolean[] brakeMode = {
            true,
            false,
            true,
            false
    };

    private final AHRS navX = new AHRS(SerialPort.Port.kMXP);

    private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();


    // Temporary until CTRE supports FalconFX in WPILib Sim
    private final TalonSRX[] simMotors = new TalonSRX[4];

    public DifferentialDrivetrainSim m_drivetrainSimulator;
    private ADXRS450_GyroSim m_gyroAngleSim;

    public DriveTrain() {
        // Set up DriveTrain motors
        configureCtreMotors(driveMotors);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeadingDegrees()));


        if (RobotBase.isSimulation()) {
            for (int i = 0; i < 4; i++)
                simMotors[i] = new TalonSRX(24 + i);
            configureCtreMotors(simMotors);
            simMotors[0].setSensorPhase(true);
            simMotors[2].setSensorPhase(false);

            m_drivetrainSimulator = new DifferentialDrivetrainSim(
                    Constants.DriveTrain.kDrivetrainPlant,
                    Constants.DriveTrain.kDriveGearbox,
                    Constants.DriveTrain.kDriveGearing,
                    Constants.DriveTrain.kTrackWidthMeters,
                    Constants.DriveTrain.kWheelDiameterMeters / 2.0,
                    null);//VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));


            m_gyroAngleSim = new ADXRS450_GyroSim(m_gyro);
        }
        SmartDashboard.putData("DT Subsystem", this);
    }

    public void configureCtreMotors(BaseTalon... motors) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].configFactoryDefault();

            motors[i].configOpenloopRamp(0.1);
            motors[i].configClosedloopRamp(0.1);
            motors[i].setNeutralMode(NeutralMode.Coast);
            motors[i].configForwardSoftLimitEnable(false);
            motors[i].configReverseSoftLimitEnable(false);

            if (motors[i] instanceof TalonFX) {
                driveMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
                driveMotors[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            } else if (motors[i] instanceof TalonSRX) {
                simMotors[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
                simMotors[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
            }
        }

        motors[0].setInverted(true);
        motors[1].setInverted(true);
        motors[2].setInverted(false);
        motors[3].setInverted(false);

        motors[0].setSensorPhase(false);
        motors[2].setSensorPhase(false);

        motors[1].set(ControlMode.Follower, driveMotors[0].getDeviceID());
        motors[3].set(ControlMode.Follower, driveMotors[2].getDeviceID());
        motors[1].setNeutralMode(NeutralMode.Brake);
        motors[3].setNeutralMode(NeutralMode.Brake);

        motors[1].configOpenloopRamp(0);
        motors[3].configOpenloopRamp(0);
    }

    public double getEncoderCount(int sensorIndex) {
        return driveMotors[sensorIndex].getSelectedSensorPosition();
    }

    public double getHeadingDegrees() {
        if (RobotBase.isReal())
            return Math.IEEEremainder(-navX.getAngle(), 360);
        else
            return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveTrain.kGyroReversed ? -1.0 : 1.0);
    }

    public void resetAngle() {
        navX.zeroYaw();
    }

    public void setNavXOffsetDegrees(double angle) {
        navX.setAngleAdjustment(angle);
    }

    public double getWheelDistanceMeters(int sensorIndex) {

        if (RobotBase.isReal())
            return driveMotors[sensorIndex].getSelectedSensorPosition() * Constants.DriveTrain.kEncoderDistancePerPulseMeters;
        else {
            return simMotors[sensorIndex].getSelectedSensorPosition() * Constants.DriveTrain.kEncoderDistancePerPulseMetersSim;
        }
    }

    public double getMotorInputCurrentAmps(int motorIndex) {
        return driveMotors[motorIndex].getSupplyCurrent();
    }

    public void resetEncoderCounts() {
        driveMotors[0].setSelectedSensorPosition(0);
        driveMotors[2].setSelectedSensorPosition(0);
        if (RobotBase.isSimulation()) {
            simMotors[0].getSimCollection().setQuadratureRawPosition(0);
            simMotors[2].getSimCollection().setQuadratureRawPosition(0);
        }
    }

    public void setMotorArcadeDrive(double throttle, double turn) {
        double leftPWM = throttle + turn;
        double rightPWM = throttle - turn;

        // Normalization
        double magnitude = Math.max(Math.abs(leftPWM), Math.abs(rightPWM));
        if (magnitude > 1.0) {
            leftPWM *= 1.0 / magnitude;
            rightPWM *= 1.0 / magnitude;
        }

        setMotorVelocityMetersPerSecond(leftPWM * Constants.DriveTrain.kMaxVelocityMetersPerSecond, rightPWM * Constants.DriveTrain.kMaxVelocityMetersPerSecond);
    }

    public void setMotorTankDrive(double leftOutput, double rightOutput) {
        setMotorVelocityMetersPerSecond(leftOutput * Constants.DriveTrain.kMaxVelocityMetersPerSecond, rightOutput * Constants.DriveTrain.kMaxVelocityMetersPerSecond);
    }

    public void setVoltageOutput(double leftVoltage, double rightVoltage) {
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVoltage), Math.abs(rightVoltage))
                > batteryVoltage) {
            leftVoltage *= batteryVoltage / 12.0;
            rightVoltage *= batteryVoltage / 12.0;
        }
        SmartDashboardTab.putNumber("DriveTrain", "Left Voltage", leftVoltage);
        SmartDashboardTab.putNumber("DriveTrain", "Right Voltage", rightVoltage);

        setMotorPercentOutput(leftVoltage / batteryVoltage, rightVoltage / batteryVoltage);

    }

    private void setMotorPercentOutput(double leftOutput, double rightOutput) {
        m_leftOutput = leftOutput;
        m_rightOutput = rightOutput;
        driveMotors[0].set(ControlMode.PercentOutput, leftOutput);
        driveMotors[2].set(ControlMode.PercentOutput, rightOutput);

        if (RobotBase.isSimulation()) {
            simMotors[0].set(ControlMode.PercentOutput, leftOutput);
            simMotors[2].set(ControlMode.PercentOutput, rightOutput);
        }
    }

    public void setMotorVelocityMetersPerSecond(double leftSpeed, double rightSpeed) {
        m_leftOutput = leftSpeed / Constants.DriveTrain.kMaxVelocityMetersPerSecond;
        m_rightOutput = rightSpeed / Constants.DriveTrain.kMaxVelocityMetersPerSecond;
        driveMotors[0].set(ControlMode.Velocity, m_leftOutput / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10), DemandType.ArbitraryFeedForward, feedforward.calculate(m_leftOutput));
        driveMotors[2].set(ControlMode.Velocity, m_rightOutput / (Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10), DemandType.ArbitraryFeedForward, feedforward.calculate(m_rightOutput));
    }

    /**
     * Sets drivetrain motors to coast/brake
     *
     * @param mode 2 = all coast, 1 = all brake, 0 = half and half
     */
    public void setDriveTrainNeutralMode(int mode) {
        switch (mode) {
            case 2:
                for (var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Coast);
                for (var brakeMode : brakeMode)
                    brakeMode = false;
                break;
            case 1:
                for (var motor : driveMotors)
                    motor.setNeutralMode(NeutralMode.Brake);
                for (var brakeMode : brakeMode)
                    brakeMode = true;
                break;
            case 0:
            default:
                driveMotors[0].setNeutralMode(NeutralMode.Brake);
                driveMotors[1].setNeutralMode(NeutralMode.Coast);
                driveMotors[2].setNeutralMode(NeutralMode.Brake);
                driveMotors[3].setNeutralMode(NeutralMode.Coast);
                brakeMode[0] = true;
                brakeMode[1] = false;
                brakeMode[2] = true;
                brakeMode[3] = false;
                break;
        }
    }

    public DifferentialDriveWheelSpeeds getSpeedsMetersPerSecond() {
        double leftMetersPerSecond = 0, rightMetersPerSecond = 0;

        if (RobotBase.isReal()) {
//             getSelectedSensorVelocity() returns values in units per 100ms. Need to convert value to RPS
            leftMetersPerSecond = driveMotors[0].getSelectedSensorVelocity() * Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10.0;
            rightMetersPerSecond = driveMotors[2].getSelectedSensorVelocity() * Constants.DriveTrain.kEncoderDistancePerPulseMeters * 10.0;
        } else {
            leftMetersPerSecond = driveMotors[0].getSelectedSensorVelocity() * Constants.DriveTrain.kEncoderDistancePerPulseMetersSim * 10.0;
            rightMetersPerSecond = driveMotors[2].getSelectedSensorVelocity() * Constants.DriveTrain.kEncoderDistancePerPulseMetersSim * 10.0;
        }
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
    }

    public double getTravelDistanceMeters() {
        double leftMeters, rightMeters;

        if (RobotBase.isReal()) {
            leftMeters = (driveMotors[0].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI * Constants.DriveTrain.kWheelDiameterMeters;
            rightMeters = (driveMotors[2].getSelectedSensorPosition() * 10.0 / 2048) * gearRatio * Math.PI * Constants.DriveTrain.kWheelDiameterMeters;
        } else {
            leftMeters = (simMotors[0].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI * Constants.DriveTrain.kWheelDiameterMeters;
            rightMeters = (simMotors[2].getSelectedSensorPosition() * 10.0 / 4096) * Math.PI * Constants.DriveTrain.kWheelDiameterMeters;
        }
        return (leftMeters + rightMeters) / 2.0;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public Pose2d getRobotPoseMeters() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveKinematics getDriveTrainKinematics() {
        return kinematics;
    }

    public PIDController getLeftPIDController() {
        return leftPIDController;
    }

    public PIDController getRightPIDController() {
        return rightPIDController;
    }

    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        if (RobotBase.isSimulation()) {
            resetEncoderCounts();
            m_drivetrainSimulator.setPose(pose);
        }

        odometry.resetPosition(pose, rotation);
        resetEncoderCounts();
    }

    private void initShuffleboardValues() {
        // Need to verify that this works again
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("Left Encoder", () -> getEncoderCount(0));
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("Right Encoder", () -> getEncoderCount(2));
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("xCoordinate", () ->
                Units.metersToFeet(getRobotPoseMeters().getTranslation().getX()));
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("yCoordinate", () ->
                Units.metersToFeet(getRobotPoseMeters().getTranslation().getY()));
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("Angle", () ->
                getRobotPoseMeters().getRotation().getDegrees());
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("leftSpeed", () ->
                Units.metersToFeet(getSpeedsMetersPerSecond().leftMetersPerSecond));
        Shuffleboard.getTab("Constants.DriveTrain Train").addNumber("rightSpeed", () ->
                Units.metersToFeet(getSpeedsMetersPerSecond().rightMetersPerSecond));


        Shuffleboard.getTab("Turret").addNumber("Robot Angle", navX::getAngle);
    }

    private void updateSmartDashboard() {
        if (RobotBase.isReal()) {
            SmartDashboardTab.putNumber("DriveTrain", "Left Distance", getWheelDistanceMeters(0));
            SmartDashboardTab.putNumber("DriveTrain", "Right Distance", getWheelDistanceMeters(2));
            SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
                    Units.metersToFeet(getRobotPoseMeters().getTranslation().getX()));
            SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
                    Units.metersToFeet(getRobotPoseMeters().getTranslation().getY()));
            SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPoseMeters().getRotation().getDegrees());
            SmartDashboardTab.putNumber("DriveTrain", "leftSpeed",
                    Units.metersToFeet(getSpeedsMetersPerSecond().leftMetersPerSecond));
            SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
                    Units.metersToFeet(getSpeedsMetersPerSecond().rightMetersPerSecond));

            SmartDashboardTab.putNumber("Turret", "Robot Angle", getHeadingDegrees());
        } else {
            SmartDashboardTab.putNumber("DriveTrain", "Left Encoder", getEncoderCount(0));
            SmartDashboardTab.putNumber("DriveTrain", "Right Encoder", getEncoderCount(2));
            SmartDashboardTab.putNumber("DriveTrain", "xCoordinate",
                    Units.metersToFeet(getRobotPoseMeters().getTranslation().getX()));
            SmartDashboardTab.putNumber("DriveTrain", "yCoordinate",
                    Units.metersToFeet(getRobotPoseMeters().getTranslation().getY()));
            SmartDashboardTab.putNumber("DriveTrain", "Angle", getRobotPoseMeters().getRotation().getDegrees());
            SmartDashboardTab.putNumber("DriveTrain", "leftSpeed",
                    Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));
            SmartDashboardTab.putNumber("DriveTrain", "rightSpeed",
                    Units.metersToFeet(m_drivetrainSimulator.getLeftVelocityMetersPerSecond()));

            SmartDashboardTab.putNumber("Turret", "Robot Angle", getHeadingDegrees());
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(Rotation2d.fromDegrees(getHeadingDegrees()), getWheelDistanceMeters(0), getWheelDistanceMeters(2));
        updateSmartDashboard();
    }

    public double getDrawnCurrentAmps() {
        return m_drivetrainSimulator.getCurrentDrawAmps();
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.

        m_drivetrainSimulator.setInputs(m_leftOutput * RobotController.getBatteryVoltage(),
                m_rightOutput * RobotController.getBatteryVoltage());
        m_drivetrainSimulator.update(0.040);

        // For CTRE devices, you must call this function periodically for simulation
        Unmanaged.feedEnable(40);
        simMotors[0].getSimCollection().setQuadratureRawPosition((int) (m_drivetrainSimulator.getLeftPositionMeters() / Constants.DriveTrain.kEncoderDistancePerPulseMetersSim));
        simMotors[0].getSimCollection().setQuadratureVelocity((int) (m_drivetrainSimulator.getLeftVelocityMetersPerSecond() / (Constants.DriveTrain.kEncoderDistancePerPulseMetersSim * 10.0)));
        simMotors[2].getSimCollection().setQuadratureRawPosition((int) (m_drivetrainSimulator.getRightPositionMeters() / Constants.DriveTrain.kEncoderDistancePerPulseMetersSim));
        simMotors[2].getSimCollection().setQuadratureVelocity((int) (m_drivetrainSimulator.getRightVelocityMetersPerSecond() / (Constants.DriveTrain.kEncoderDistancePerPulseMetersSim * 10.0)));
        m_gyroAngleSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());

        SmartDashboard.putNumber("Robot Angle", getHeadingDegrees());
        SmartDashboard.putNumber("L Encoder Count", simMotors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("R Encoder Count", simMotors[2].getSelectedSensorPosition());
        SmartDashboard.putNumber("L Encoder Rate", simMotors[0].getSelectedSensorVelocity());
        SmartDashboard.putNumber("R Encoder Rate", simMotors[2].getSelectedSensorVelocity());

        SmartDashboard.putNumber("L Output", m_leftOutput);
        SmartDashboard.putNumber("R Output", m_rightOutput);
        SmartDashboard.putNumber("L Encoder Distance", getWheelDistanceMeters(0));
        SmartDashboard.putNumber("R Encoder Distance", getWheelDistanceMeters(2));
//        SmartDashboard.putNumber("L Encoder Count", m_leftEncoder.get());
//        SmartDashboard.putNumber("R Encoder Count", m_rightEncoder.get());
//        SmartDashboard.putNumber("L Encoder Rate", m_leftEncoder.getRate());
//        SmartDashboard.putNumber("R Encoder Rate", m_rightEncoder.getRate());

        SmartDashboard.putBoolean("CTRE Feed Enabled", Unmanaged.getEnableState());
    }
}