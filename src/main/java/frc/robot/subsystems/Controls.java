package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.controls.SetAllianceColor;

public class Controls extends SubsystemBase {
  private boolean overrideFmsAlliance;
  private DriverStation.Alliance overrideFmsAllianceColor;

  public Controls() {
    initSmartDashboard();
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
    DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;
    if (overrideFmsAlliance) {
      alliance = overrideFmsAllianceColor;
    } else if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      if (alliance != DriverStation.Alliance.Blue || alliance != DriverStation.Alliance.Red) {
        // System.out.println("Vision Subsystem Error: Invalid Alliance Color. Defaulting to Red");
        alliance = DriverStation.Alliance.Red;
      }
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

  /** Sets whether or not to ignore the FMS to determine alliance color. */
  public void setOverrideFmsAlliance(boolean state) {
    overrideFmsAlliance = state;
  }

  /** Sets the alliance color to use */
  public void setOverrideFmsAllianceColor(DriverStation.Alliance color) {
    overrideFmsAllianceColor = color;
  }

  /** Initializes values on SmartDashboard */
  private void initSmartDashboard() {
    Shuffleboard.getTab("Controls")
        .add("Set Alliance Red", new SetAllianceColor(this, DriverStation.Alliance.Red));
    Shuffleboard.getTab("Controls")
        .add("Set Alliance Blue", new SetAllianceColor(this, DriverStation.Alliance.Blue));

    Shuffleboard.getTab("Controls").add("Alliance", getAllianceColor().toString());
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
