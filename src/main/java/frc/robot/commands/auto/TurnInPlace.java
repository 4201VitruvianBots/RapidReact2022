package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TurnInPlace extends CommandBase {
    private DriveTrain m_driveTrain;
    private double m_setpoint;

    public double kP = 0.09;
    public double kI = 0;
    public double kD = 0.011;

    private double degreesTolerance = 3;
    private double degreesPerSecondTolerance = 3;
    private double kDt = 0.02;

    private PIDController m_controller = new PIDController(kP, kI, kD, kDt);
    private SimpleMotorFeedforward m_feedforward;
    private double m_startTime;
    private double m_timeout = 2;

    double distancePerAngle;
    double maxVel;  // 2.167780360340067 m/s
    double lastTimestamp;
    double lastAngle;

    boolean latch = true;
    /**
     * Creates a new ExampleCommand.
     *
     * @param driveTrain The subsystem used by this command.
     */
    public TurnInPlace(DriveTrain driveTrain, double targetAngle) {
        m_driveTrain = driveTrain;
        m_setpoint = targetAngle;

        m_feedforward = m_driveTrain.getFeedforward();

        m_controller.setSetpoint(targetAngle);
        m_controller.enableContinuousInput(-180, 180);
        m_controller.setTolerance(degreesTolerance, degreesPerSecondTolerance);
        m_controller.setIntegratorRange(-30.0, 30.0);

        distancePerAngle = Constants.DriveTrain.kTrackWidthMeters * Math.PI / 360;

        m_driveTrain = driveTrain;
    }

    @Override
    public void initialize() {
        m_startTime = lastTimestamp = Timer.getFPGATimestamp();
        lastAngle = m_driveTrain.getHeadingDegrees();
        latch = true;
    }

    @Override
    public void execute() {
        double pid_output = m_controller.calculate(m_driveTrain.getHeadingDegrees());
        pid_output = Math.max(Math.min(pid_output, 0.6), -0.6);
        m_driveTrain.setMotorArcadeDrive(0, pid_output);
        if(Math.abs(m_driveTrain.getHeadingDegrees()) >= m_setpoint  && latch) {
            // 0.417952 seconds
            // SmartDashboard.putNumber("MinTime:", Timer.getFPGATimestamp() - lastTimestamp);
            latch = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // SmartDashboard.putNumber("RunTime:", Timer.getFPGATimestamp() - m_startTime);
        m_driveTrain.setVoltageOutput(0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
