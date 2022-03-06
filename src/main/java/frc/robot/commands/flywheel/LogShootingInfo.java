// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/** Logs shooter and indexer information to a log file. */
public class LogShootingInfo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Flywheel m_flywheel;

  private final Indexer m_indexer;
  private FileWriter m_logWriter;

  /**
   * Logs shooter and indexer information to a log file.
   *
   * @param flywheel The flywheel used by this command.
   * @param indexer The indexer used by this command.
   */
  public LogShootingInfo(Flywheel flywheel, Indexer indexer) {
    m_flywheel = flywheel;
    m_indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      File logFile =
          new File(
              "/home/lvuser/frc/shooter_log/"
                  + m_flywheel.getTestingSessionName()
                  + "/shooter_log_"
                  + Timer.getFPGATimestamp()
                  + ".csv");
      logFile.createNewFile();
      m_logWriter = new FileWriter(logFile);
    } catch (IOException e) {
      e.printStackTrace();
      this.cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      m_logWriter.write(
          m_flywheel.getRPM(0)
              + ","
              + m_flywheel.getRPM(1)
              + ","
              + m_flywheel.getSetpointRPM()
              + ","
              + m_indexer.getIndexerOutput()
              + ","
              + m_indexer.getKickerOutput()
              + "\n");
    } catch (IOException e) {
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      try {
        m_logWriter.close();
      } catch (IOException e) {
        return;
      } catch (NullPointerException e) { // If file could not be created
        return;
      }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
