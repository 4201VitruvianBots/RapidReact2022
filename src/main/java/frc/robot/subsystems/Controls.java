package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Controls extends SubsystemBase {

  /**
   * Returns the robot's current alliance color
   *
   * @return Returns the current alliance color.
   */
  public DriverStation.Alliance getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance != DriverStation.Alliance.Blue || alliance != DriverStation.Alliance.Red) {
      System.out.println("Vision Subsystem Error: Invalid Alliance Color. Defaulting to Red");
      alliance = DriverStation.Alliance.Red;
    }

    return alliance;
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    SmartDashboardTab.putString("Controls", "Alliance", getAllianceColor().toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
