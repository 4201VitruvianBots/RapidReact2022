package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.controls.OverrideAllianceColor;

public class Controls extends SubsystemBase {
  private DriverStation.Alliance allianceColor;
  private DriverStation.Alliance allianceColorOverride;

  private boolean overrideAllianceColor = false;

  public Controls() {
    updateAllianceColor();
  }

  /**
   * Updates the alliance color from the driverstation API. If no valid color is returned, it will default to Red.
   *
   */
  public void updateAllianceColor() {
    if(!overrideAllianceColor) {
      allianceColor = DriverStation.getAlliance();
      if (allianceColor != DriverStation.Alliance.Blue || allianceColor != DriverStation.Alliance.Red) {
        System.out.println("Vision Subsystem Error: Invalid Alliance Color. Defaulting to Red");
        allianceColor = DriverStation.Alliance.Red;
      }
    } else
      allianceColor = allianceColorOverride;
  }
  /**
   * Returns the robot's current alliance color
   *
   * @return Returns the current alliance color.
   */
  public DriverStation.Alliance getAllianceColor() {
    if(overrideAllianceColor)
      return allianceColorOverride;
    else
      return allianceColor;
  }

  /**
   * Sets the robot's current alliance color
   *
   */
  public void overrideAllianceColor(DriverStation.Alliance alliance) {
    this.allianceColorOverride = alliance;
    this.overrideAllianceColor = true;
  }
  /**
   * Sets the robot's current alliance color
   *
   */
  public void setAllianceColorOverrideState(boolean state) {
    this.overrideAllianceColor = state;
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    SmartDashboardTab.putString("Controls", "Alliance", getAllianceColor().toString());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();

    if(DriverStation.getAlliance() == DriverStation.Alliance.Invalid)
      updateAllianceColor();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
