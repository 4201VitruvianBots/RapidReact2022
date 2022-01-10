package frc.robot.simulation;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Outtake;

public class Powercell {
    boolean wasShot;
    Pose2d ballPose = new Pose2d();
    private double ballZPos = 0; // Height of the center of the ball above the ground
    private double ballZVel = 0; // Vertical velocity of the ball
    private Pose2d ballXYVel = new Pose2d(); // X and Y velocity of the ball while in the air
    double m_lastTimestamp;
    private int ballState = 0;


    String m_name;

    public Powercell(String name) {
        m_name = name;
    }

    public String getName() {
        return m_name;
    }

    public int getBallState() {
        return ballState;
    }

    public boolean getBallShotState() {
        return wasShot;
    }

    public void setBallState(int state) {
        ballState = state;
    }

    public void setBallShotState(boolean shotState) {
        wasShot = shotState;
    }

    public void setBallPose(Pose2d pose) {
        ballPose = pose;
    }

    public Pose2d getBallPose() {
        return ballPose;
    }

    public void moveBallZPos(double deltaZ) {
        ballZPos += deltaZ;
    }

    public double getBallZPos() {
        return ballZPos;
    }

    public void setBallZVel(double m_ballZVel) {
        ballZVel = m_ballZVel;
    }

    public double getBallZVel() {
        return ballZVel;
    }

    public void setBallXYVel(Pose2d m_ballXYVel) {
        ballXYVel = m_ballXYVel;
    }

    public Pose2d getBallXYVel() {
        return ballXYVel;
    }

    public void setLastTimestamp(double timestamp) {
        m_lastTimestamp = timestamp;
    }

    public double getLastTimestamp() {
        return m_lastTimestamp;
    }

    public void simulationPeriodic() {
//        updateBallState();
    }
}
