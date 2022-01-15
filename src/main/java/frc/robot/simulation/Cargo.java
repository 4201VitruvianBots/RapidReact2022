package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Sim.BallState;

public class Cargo {
    boolean wasShot;
    Pose2d ballPose = new Pose2d();
    private Pose2d ballVel = new Pose2d();
    double m_lastTimestamp;
    private BallState ballState = 0;


    String m_name;

    public Cargo(String name) {
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

    public void setBallVel(Pose2d m_ballVel) {
        ballVel = m_ballVel;
    }

    public Pose2d getBallVel() {
        return ballVel;
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
