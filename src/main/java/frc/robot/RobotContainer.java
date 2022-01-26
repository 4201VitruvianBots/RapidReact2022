// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.auto.TestPath;
import frc.robot.commands.driveTrain.SetArcadeDrive;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.led.GetSubsystemStates;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final FieldSim m_fieldSim = new FieldSim(m_driveTrain);
  private final Controls m_controls = new Controls();
  private final Vision m_vision = new Vision(m_controls);
  private final Flywheel m_flywheel = new Flywheel(m_vision);
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final LED m_led = new LED();
  private final Climber m_climber = new Climber();

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static Joystick xBoxController = new Joystick(Constants.USB.xBoxController);

  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;
  public static boolean allianceColorBlue;
  public static boolean allianceColorRed;

  public static enum CommandSelector {
    BLUE_ALLIANCE, // 01
    RED_ALLIANCE
  }

  public final SendableChooser<CommandSelector> m_allianceChooser =
      new SendableChooser<CommandSelector>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    /** Sets the AllianceColor */
    // for (CommandSelector command : CommandSelector.values()) {
    //   if (command == CommandSelector.BLUE_ALLIANCE) {
    //     m_allianceChooser.setDefaultOption(command.toString(), command);
    //     allianceColorBlue = true;
    //     allianceColorRed = false;
    //   }else{
    //     m_allianceChooser.addOption(command.toString(), command);
    //     allianceColorBlue = false;
    //     allianceColorRed = true;
    //   }
    // }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 45));

    xBoxLeftTrigger = new Button(() -> xBoxController.getRawButton(2));
    xBoxRightTrigger = new Button(() -> xBoxController.getRawButton(3));
    xBoxLeftTrigger.whenPressed(new IntakePiston(m_intake, true)); // Left trigger: Extend intake
    xBoxLeftTrigger.whenReleased(new IntakePiston(m_intake, false)); // Left trigger: Retract intake
    xBoxLeftTrigger.whileHeld(
        new RunIntake(m_intake, m_indexer)); // Left trigger: intake & carousel
  }

  public void initializeSubsystems() {
    m_driveTrain.setDefaultCommand(
        new SetArcadeDrive(m_driveTrain, leftJoystick::getY, rightJoystick::getX));
    m_led.setDefaultCommand(
        new GetSubsystemStates(m_led, m_intake, m_vision, m_flywheel, m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new TestPath(m_driveTrain, m_fieldSim);
  }

  public void robotPeriodic() {}

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void teleopInit() {}

  public void teleopPeriodic() {}

  public void autonomousInit() {}

  public void autonomousPeriodic() {}

  public void simulationInit() {
    m_fieldSim.initSim();
  }

  public void simulationPeriodic() {
    m_fieldSim.simulationPeriodic();
  }
}
