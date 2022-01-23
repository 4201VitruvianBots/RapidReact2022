// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static java.util.Map.entry;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.auto.OneBallAuto;
import frc.robot.commands.auto.TestPath;
import frc.robot.commands.auto.ThreeBallAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.driveTrain.DriveForwardDistance;
import frc.robot.commands.driveTrain.SetArcadeDrive;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Turret m_turret = new Turret(m_driveTrain);
  private final Vision m_vision = new Vision();
  private final Flywheel m_flywheel = new Flywheel(m_vision);
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();
  
  private final FieldSim m_fieldSim = new FieldSim(m_driveTrain, m_intake);

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static Joystick xBoxController = new Joystick(Constants.USB.xBoxController);
  static Joystick testController = new Joystick(3);

  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;

  public static enum CommandSelector {
    ONE_BALL_AUTO,
    TWO_BALL_AUTO,
    THREE_BALL_AUTO,
    TEST_PATH,
    DRIVE_FORWARD
  }

  private final SendableChooser<CommandSelector> m_autoChooser =
      new SendableChooser<CommandSelector>();
  private final SelectCommand m_autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Setup auto chooser
    for (CommandSelector command : CommandSelector.values()) {
      if (command == CommandSelector.ONE_BALL_AUTO)
        m_autoChooser.setDefaultOption(command.toString(), command);
      else m_autoChooser.addOption(command.toString(), command);
    }

    SmartDashboard.putData("Selected Auto", m_autoChooser);

    m_autoCommand =
        new SelectCommand(
            Map.ofEntries(
                entry(CommandSelector.ONE_BALL_AUTO,new OneBallAuto(m_driveTrain, m_fieldSim, m_indexer, m_flywheel, m_vision)),
                entry(CommandSelector.TWO_BALL_AUTO,new TwoBallAuto(m_driveTrain, m_fieldSim, m_intake, m_flywheel, m_indexer, m_vision)),
                entry(CommandSelector.THREE_BALL_AUTO,new ThreeBallAuto(m_driveTrain,m_fieldSim,m_intake,m_indexer,m_flywheel,m_turret,m_vision)),
                entry(CommandSelector.TEST_PATH, new TestPath(m_driveTrain, m_fieldSim)),
                entry(CommandSelector.DRIVE_FORWARD, new DriveForwardDistance(m_driveTrain, m_fieldSim, 4))),
            m_autoChooser::getSelected);

    initializeSubsystems();

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
  }

  public void initializeSubsystems() {
    m_driveTrain.setDefaultCommand(
        new SetArcadeDrive(m_driveTrain, leftJoystick::getY, rightJoystick::getX));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public void robotPeriodic() {}

  public void disabledInit() {
    m_driveTrain.setDriveTrainNeutralMode(DriveTrainNeutralMode.COAST);
    m_driveTrain.setMotorTankDrive(0, 0);
  }

  public void disabledPeriodic() {}

  public void teleopInit() {
    m_driveTrain.setDriveTrainNeutralMode(DriveTrainNeutralMode.COAST);
  }

  public void teleopPeriodic() {}

  public void autonomousInit() {
    if (RobotBase.isReal()) {
      m_driveTrain.resetEncoderCounts();
      m_driveTrain.resetOdometry(
          m_driveTrain.getRobotPoseMeters(), m_fieldSim.getRobotPose().getRotation());
      m_driveTrain.resetAngle();
    } else {
      m_fieldSim.initSim();
      m_driveTrain.resetEncoderCounts();
      m_driveTrain.resetOdometry(
          m_fieldSim.getRobotPose(), m_fieldSim.getRobotPose().getRotation());
      m_driveTrain.resetAngle();
    }
  }

  public void autonomousPeriodic() {}

  public void simulationInit() {
    m_fieldSim.initSim();
  }

  public void simulationPeriodic() {
    m_fieldSim.simulationPeriodic();
  }
}
