/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.simulation;

import javax.xml.catalog.CatalogResolver;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Sim.BallState;

/**
 * An example command that uses an example subsystem.
 */
public class SimulationShoot extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private double startTime, m_shootTimeout;
    private boolean timerStart;
    FieldSim m_fieldSim;
    private static double lastShotTime;

    private boolean m_continuous;

    private Pose2d m_shootVel;
    /**
     * Creates a new ExampleCommand.
     *
     * @param RobotContainer.m_shooter The subsystem used by this command.
     */
    public SimulationShoot(FieldSim fieldSim, boolean continuous) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_fieldSim = fieldSim;
        m_continuous = continuous;
    }

    public void setVelocity(Pose2d shootVel) {
        m_shootVel = shootVel;
        //System.out.println("x-vel: " + xyShootVel.getX() + "\ty-vel: " + xyShootVel.getY() + " \tz-vel: " + zShootVel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // double currentTime = RobotController.getFPGATime();
        // Shoot only every 80ms
        // if(((currentTime - lastShotTime) / 1e6) > 0.080) {
            for(Cargo p: m_fieldSim.getCargo()) {
                if(p.getBallState() == BallState.IN_ROBOT && !p.getBallShotState()) {
                    p.setBallShotState(true);
                    //lastShotTime = currentTime;
                    p.setBallVel(m_shootVel);
                    break;
                }
           // }
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !m_continuous;
    }
}
