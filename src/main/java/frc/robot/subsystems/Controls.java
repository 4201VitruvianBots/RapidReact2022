package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.controls.OverrideAllianceColor;

public class Controls extends SubsystemBase {
  private boolean overrideFmsAlliance;
  private DriverStation.Alliance overrideFmsAllianceColor;
  private DriverStation.Alliance alliance = DriverStation.Alliance.Invalid;
  // private PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public Controls() {
    initSmartDashboard();
  }
  /**
   * Returns the robot's current alliance color
   *
   * @return Returns the current alliance color.
   */
  public void updateAllianceColor() {
    if (overrideFmsAlliance) {
      alliance = overrideFmsAllianceColor;
    } else if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      if (alliance != DriverStation.Alliance.Blue && alliance != DriverStation.Alliance.Red) {
        //         System.out.println("Vision Subsystem Error: Invalid Alliance Color. Defaulting to
        // Red");
        alliance = DriverStation.Alliance.Red;
      }
    }
  }

  public DriverStation.Alliance getAllianceColor() {
    return alliance;
  }

  /**
   * Returns true when the alliance color is not Blue`
   *
   * @return Returns the current alliance color.
   */
  public boolean getAllianceColorBoolean() {
    return alliance != DriverStation.Alliance.Blue;
  }

  /** Sets whether or not to ignore the FMS to determine alliance color. */
  public void setOverrideFmsAlliance(boolean state) {
    overrideFmsAlliance = state;
  }

  /** Sets the alliance color to use */
  public void setOverrideFmsAllianceColor(DriverStation.Alliance color) {
    overrideFmsAllianceColor = color;
  }

  public void setPDHChannel(boolean on) {
    // pdh.setSwitchableChannel(on);
  }

  /** Initializes values on SmartDashboard */
  private void initSmartDashboard() {
    //    Shuffleboard.getTab("SmartDashboard")
    //            .add("Alliance", getAllianceColorBoolean())
    //            .withWidget(BuiltInWidgets.kBooleanBox)
    //            .withProperties(Map.of("Color when true", "#FF0000", "Color when false",
    // "#0000FF"));
    Shuffleboard.getTab("Controls")
        .add("Set Alliance Red", new OverrideAllianceColor(this, DriverStation.Alliance.Red));
    Shuffleboard.getTab("Controls")
        .add("Set Alliance Blue", new OverrideAllianceColor(this, DriverStation.Alliance.Blue));

    Shuffleboard.getTab("Controls")
        .addString("alliance_string", () -> getAllianceColor().toString());
  }

  /** Sends values to SmartDashboard */
  private void updateSmartDashboard() {
    SmartDashboard.putBoolean("Alliance", getAllianceColorBoolean());
    SmartDashboardTab.putString("Controls", "alliance_string", getAllianceColor().toString());
  }

  @Override
  public void periodic() {
    updateAllianceColor();
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
