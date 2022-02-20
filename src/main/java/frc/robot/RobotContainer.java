// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.commands.auto.FourBallAuto;
import frc.robot.commands.auto.GroupThreeBallAuto;
import frc.robot.commands.auto.GroupThreeBallAutoLowerHub;
import frc.robot.commands.auto.OneBallAuto;
import frc.robot.commands.auto.TestPath;
import frc.robot.commands.auto.TwoBallAutoUpper;
import frc.robot.commands.auto.TwoBallAutoLower;
import frc.robot.commands.climber.EngageHighClimb;
import frc.robot.commands.climber.SetClimbState;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.controls.SetFloodlight;
import frc.robot.commands.driveTrain.AlignToCargo;
import frc.robot.commands.driveTrain.AlignToLaunchpad;
import frc.robot.commands.driveTrain.DriveForwardDistance;
import frc.robot.commands.driveTrain.SetArcadeDrive;
import frc.robot.commands.flywheel.SetRpmSetpoint;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.ReverseIntakeIndexer;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.led.GetSubsystemStates;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Controls m_controls = new Controls();
  private final Turret m_turret = new Turret(m_driveTrain);
  private final Vision m_vision = new Vision(m_controls);
  private final Flywheel m_flywheel = new Flywheel(m_vision);
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final LED m_led = new LED();
  private final Climber m_climber = new Climber();

  private final FieldSim m_fieldSim = new FieldSim(m_driveTrain, m_intake);

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);

  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[8];
  public Button xBoxLeftTrigger, xBoxRightTrigger;
  // public static boolean allianceColorBlue;
  // public static boolean allianceColorRed;
  public static enum CommandSelector {
    BLUE_ALLIANCE, // 01
    RED_ALLIANCE
  }

  public final SendableChooser<CommandSelector> m_allianceChooser =
      new SendableChooser<CommandSelector>();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Setup auto chooser
    m_autoChooser.setDefaultOption(
        "Drive Forward", new DriveForwardDistance(m_driveTrain, m_fieldSim, 3));
    m_autoChooser.addOption(
        "One Ball Auto",
        new OneBallAuto(m_driveTrain, m_fieldSim, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "Two Ball Auto - Upper",
        new TwoBallAutoUpper(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "Two Ball Auto - Lower",
        new TwoBallAutoLower(
        m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
     m_autoChooser.addOption(
        "Two Ball Auto - UpperLowerHub",
         new TwoBallAutoUpper(
         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
     m_autoChooser.addOption(
              "Two Ball Auto - LowerLowerHub",
              new TwoBallAutoLower(
              m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
     m_autoChooser.addOption(
        "Group Three Ball Auto",
        new GroupThreeBallAuto(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
      "Group Three Ball Auto LowerHub",
      new GroupThreeBallAutoLowerHub(
          m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
  m_autoChooser.addOption(
        "Four Ball Auto",
        new FourBallAuto(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption("Test Path", new TestPath(m_driveTrain, m_fieldSim));
   
    SmartDashboard.putData("Selected Auto", m_autoChooser);

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

    rightButtons[0].whileHeld(
        new AlignToCargo(m_driveTrain, m_vision, leftJoystick::getY, rightJoystick::getX));
    rightButtons[1].whileHeld(
        new AlignToLaunchpad(m_driveTrain, m_vision, leftJoystick::getY, rightJoystick::getX));

    xBoxLeftTrigger =
        new Button(
            () -> xBoxController.getLeftTriggerAxis() > 0.05); // getTrigger());// getRawAxis(2));
    xBoxRightTrigger = new Button(() -> xBoxController.getRightTriggerAxis() > 0.05);

    xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_flywheel, m_vision, 1300));
    xBoxButtons[1].whileHeld(new SetRpmSetpoint(m_flywheel, m_vision, 2400)); // 2300
    xBoxButtons[3].whileHeld(new SetRpmSetpoint(m_flywheel, m_vision, 3500)); // 2900

    xBoxButtons[0].whileHeld(new SetFloodlight(m_controls));
    xBoxButtons[1].whileHeld(new SetFloodlight(m_controls));
    xBoxButtons[3].whileHeld(new SetFloodlight(m_controls));

    xBoxButtons[6].whenPressed(new ToggleTurretControlMode(m_turret));

    xBoxPOVButtons[4].whileHeld(new ReverseIntakeIndexer(m_intake, m_indexer));
    xBoxLeftTrigger.whileHeld(new RunIntake(m_intake, m_indexer));
    xBoxRightTrigger.whileHeld(new RunIndexer(m_indexer, m_flywheel));
    // xBoxRightTrigger.whileHeld(new LogShootingInfo(m_flywheel, m_indexer));

    xBoxButtons[2].whenPressed(new EngageHighClimb(m_climber));

    xBoxButtons[9].whileHeld(new SetClimbState(m_climber, true));

    // xBoxButtons[6].whenPressed(new SetClimbState(m_climber, true));
    // xBoxButtons[7].whenPressed(new SetClimbState(m_climber, false));
    // leftButtons[0].cancelWhenPressed(m_driveTrain.getPostAutoCommand());
    // TODO Try this if the above does not work
    // leftButtons[0].cancelWhenPressed(m_driveTrain.getCurrentCommand());
  }

  public void initializeSubsystems() {
    m_driveTrain.setDefaultCommand(
        new SetArcadeDrive(m_driveTrain, leftJoystick::getY, rightJoystick::getX));
    m_climber.setDefaultCommand(
        new SetClimberOutput(m_climber, () -> xBoxController.getRawAxis(5)));
    // m_indexer.setDefaultCommand(
    //     new ColorSensor(m_indexer, m_controls, m_intake, m_flywheel, () ->
    // xBoxRightTrigger.get()));
    m_led.setDefaultCommand(
        new GetSubsystemStates(m_led, m_intake, m_vision, m_flywheel, m_climber));
    m_turret.setDefaultCommand(
        new SetTurretSetpointFieldAbsolute(
            m_turret, m_driveTrain, m_vision, m_flywheel, m_climber, xBoxController));
  }

  public Indexer getIndexer() {
    return m_indexer;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

  public void robotPeriodic() {}

  public void disabledInit() {
    m_driveTrain.setDriveTrainNeutralMode(DriveTrainNeutralMode.COAST);
    m_driveTrain.setMotorTankDrive(0, 0);
    m_driveTrain.setPostAutoCommand(null);
  }

  public void disabledPeriodic() {}

  public void teleopInit() {
    m_driveTrain.setDriveTrainNeutralMode(DriveTrainNeutralMode.BRAKE);
    m_climber.setHoldPosition(m_climber.getElevatorClimbPosition());
    if (m_driveTrain.getPostAutoCommand() != null) {
      m_driveTrain.getPostAutoCommand().schedule(true);
    }
  }

  public void teleopPeriodic() {}

  public void autonomousInit() {
    if (RobotBase.isReal()) {
      m_driveTrain.resetEncoderCounts();
      m_driveTrain.resetOdometry(
          m_driveTrain.getRobotPoseMeters(), m_fieldSim.getRobotPose().getRotation());
      m_driveTrain.resetAngle();
      m_climber.setHoldPosition(m_climber.getElevatorClimbPosition());
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
