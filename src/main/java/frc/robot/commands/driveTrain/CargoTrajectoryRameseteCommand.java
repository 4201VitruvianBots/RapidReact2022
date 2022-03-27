package frc.robot.commands.driveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.auto.VitruvianRamseteCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class CargoTrajectoryRameseteCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_driveTrain;

  private final Vision m_vision;

  private TrajectoryConfig m_pathConfig;
  private Trajectory m_path;
  private ArrayList<Pose2d> trajectoryStates;
  private VitruvianRamseteCommand m_command;

  private Pose2d startPos, endPos;
  private Translation2d cargoPose;
  private Rotation2d endAngle;

  private boolean finished = false;

  private final Timer m_timer = new Timer();
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  /** Creates a new ExampleCommand. */
  public CargoTrajectoryRameseteCommand(DriveTrain driveTrain, Vision vision) {
    m_driveTrain = driveTrain;
    m_vision = vision;

    m_pose = m_driveTrain::getRobotPoseMeters;
    m_follower = new RamseteController();
    m_feedforward = m_driveTrain.getFeedforward();
    m_kinematics = m_driveTrain.getDriveTrainKinematics();
    m_speeds = m_driveTrain::getSpeedsMetersPerSecond;
    m_leftController = m_driveTrain.getLeftPIDController();
    m_rightController = m_driveTrain.getRightPIDController();
    m_output = m_driveTrain::setVoltageOutput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;

    startPos = m_driveTrain.getRobotPoseMeters();

    if (m_vision.getValidTarget(Constants.Vision.CAMERA_POSITION.INTAKE)) {

      /** Field relative cargo pose */
      // cargoPose = new Translation2d(cargoPose .getX(),cargoPose .getY()+(Math.signum(cargoPose
      // .getY()) * Constants.Vision.CARGO_RADIUS));
      /** Field relative cargo angle */
      cargoPose =
          m_vision
              .getCargoPositionFromRobot()
              .rotateBy(startPos.getRotation())
              .plus(startPos.getTranslation());
      Rotation2d angleToCargo =
          new Rotation2d(startPos.getX() - cargoPose.getX(), startPos.getY() - cargoPose.getY());
      endAngle = angleToCargo;
      endPos =
          new Pose2d(
              cargoPose.minus(Constants.Vision.INTAKE_TRANSLATION.rotateBy(endAngle)), endAngle);
    } else {
      finished = true;
      return;
    }

    m_pathConfig =
        new TrajectoryConfig(Units.feetToMeters(14), Units.feetToMeters(5))
            // Add kinematics to ensure max speed is actually obeyed
            .setReversed(true)
            .setKinematics(Constants.DriveTrain.kDriveKinematics)
            .setEndVelocity(0)
            .setStartVelocity(0);

    m_path = TrajectoryGenerator.generateTrajectory(startPos, List.of(), endPos, m_pathConfig);

    m_driveTrain.setCurrentTrajectory(m_path);

    // RamseteCommand Initialize
    m_prevTime = -1;
    var initialState = m_path.sample(0);
    m_prevSpeeds =
        m_kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    m_leftController.reset();
    m_rightController.reset();
  }

  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    if (m_prevTime < 0) {
      m_output.accept(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    var targetWheelSpeeds =
        m_kinematics.toWheelSpeeds(m_follower.calculate(m_pose.get(), m_path.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    double leftFeedforward =
        m_feedforward.calculate(
            leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward =
        m_feedforward.calculate(
            rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

    leftOutput =
        leftFeedforward
            + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

    rightOutput =
        rightFeedforward
            + m_rightController.calculate(m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);

    m_output.accept(leftOutput, rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();

    if (interrupted) {
      m_output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return finished || m_timer.hasElapsed(m_path.getTotalTimeSeconds());
  }
}
