package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrain;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/*  The point of this class was to do two things:
   1) To simplify the construction of a Ramsete Command to follow a path.
   2) To ensure that the command actually ends once you are at your target

   The second point was due to not being sure how the RamseteCommand would behave since it was just introduced in 2020.
   May revisit the need to specify our own end conditions in the future.
*/
public class VitruvianRamseteCommand extends RamseteCommand {
  private Trajectory m_trajectory;
  private DriveTrain m_driveTrain;
  private Supplier<Pose2d> m_pose;
  private Pose2d m_finalPose;
  private double autoDuration, autoStartTime;

  public VitruvianRamseteCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      DriveTrain driveTrain) {
    super(
        trajectory,
        pose,
        controller,
        feedforward,
        kinematics,
        wheelSpeeds,
        leftController,
        rightController,
        outputVolts,
        driveTrain);
    m_driveTrain = driveTrain;
    m_pose = pose;
    m_trajectory = trajectory;

    int trajectorySize = m_trajectory.getStates().size() - 1;
    m_finalPose = m_trajectory.getStates().get(trajectorySize).poseMeters;
  }

  @Override
  public void initialize() {
    super.initialize();
    autoStartTime = Timer.getFPGATimestamp();
    autoDuration = m_trajectory.getTotalTimeSeconds() + 1;
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboardTab.putNumber(
        "DriveTrain",
        "Velocity",
        Units.metersToFeet(
            m_driveTrain
                .getDriveTrainKinematics()
                .toChassisSpeeds(m_driveTrain.getSpeedsMetersPerSecond())
                .vxMetersPerSecond));
  }

  //    @Override
  //    public boolean isFinished() {
  //        double deltaX = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getX() -
  // m_finalPose.getX()));
  //        double deltaY = Units.metersToFeet(Math.abs(m_pose.get().getTranslation().getY() -
  // m_finalPose.getY()));
  //        double deltaRot = Math.abs(m_pose.get().getRotation().getDegrees() -
  // m_finalPose.getRotation().getDegrees());
  //        boolean isFinished = ((deltaX < .25) && (deltaY < .25) && (deltaRot < 4));
  //        SmartDashboardTab.putNumber("DriveTrain", "Ramsete Delta X", deltaX);
  //        SmartDashboardTab.putNumber("DriveTrain", "Ramsete Delta Y", deltaY);
  //        SmartDashboardTab.putNumber("DriveTrain", "Ramsete Delta Rot", deltaRot);
  //        SmartDashboardTab.putBoolean("DriveTrain", "Ramsete Command Finished", isFinished);
  //        return isFinished;
  //    }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    SmartDashboardTab.putBoolean("DriveTrain", "isRunning", false);
  }
}
