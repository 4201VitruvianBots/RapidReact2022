/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;

/**
 * Sets the robot's position
 */
public class SetOdometry extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain m_driveTrain;
    private final FieldSim m_fieldSim;
    private Pose2d m_pose2d;

    /**
     * Sets the robot's position
     * 
     * @param driveTrain Drivetrain's odometry is set
     * @param pose2d position to set odometry to
     */
    public SetOdometry(DriveTrain driveTrain, Pose2d pose2d) {
        this(driveTrain, null, pose2d);
    }

    /**
     * Sets the robot's position
     * 
     * @param driveTrain Drivetrain's odometry is set
     * @param fieldSim fieldSim to set robot's position if we're simulating the robot
     * @param pose2d position to set odometry to
     */
    public SetOdometry(DriveTrain driveTrain, FieldSim fieldSim, Pose2d pose2d) {
        if (RobotBase.isSimulation() && fieldSim == null)
            System.out.println("SetOdometry Command Error: Robot is in Simulation, but you did not add FieldSim to the argument");

        m_driveTrain = driveTrain;
        m_fieldSim = fieldSim;
        m_pose2d = pose2d;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveTrain.resetOdometry(m_pose2d, m_pose2d.getRotation());
        //m_driveTrain.setNavXOffset(m_pose2d.getRotation().getDegrees());
        if (RobotBase.isSimulation())
            m_fieldSim.resetRobotPose(m_pose2d);
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
        return true;
    }
}
