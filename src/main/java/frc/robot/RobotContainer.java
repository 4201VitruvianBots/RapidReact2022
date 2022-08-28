// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.commands.auto.*;
import frc.robot.commands.climber.EngageHighClimb;
import frc.robot.commands.climber.SetClimbState;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.driveTrain.*;
import frc.robot.commands.flywheel.SetRpmSetpoint;
import frc.robot.commands.flywheel.ShotSelecter;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.RunOnlyIndexer;
import frc.robot.commands.intake.ReverseIntakeIndexer;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.led.GetSubsystemStates;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.commands.turret.SetTurretControlMode;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.commands.turret.ToggleTurretLock;
import frc.robot.commands.vision.SetGoalLEDState;
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
  private final DataLog m_logger = DataLogManager.getLog(); // DATALOGGING

  // The robot's subsystems and commands are defined here...
  private final Controls m_controls = new Controls();
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Turret m_turret = new Turret(m_driveTrain);
  private final Vision m_vision = new Vision(m_controls, m_driveTrain, m_turret, m_logger);
  private final Flywheel m_flywheel = new Flywheel(m_vision, m_turret);
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer(m_controls);
  private final LED m_led = new LED();
  private final Climber m_climber = new Climber();

  private final FieldSim m_fieldSim = new FieldSim(m_driveTrain, m_turret, m_vision, m_intake);

  DoubleLogEntry indexerRPMLog;
  DoubleLogEntry flywheelRPMLog;
  DoubleLogEntry kickerRPMLog;
  StringLogEntry poseLog;

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);

  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[4];
  public Button xBoxLeftTrigger, xBoxRightTrigger;
  // public static boolean allianceColorBlue;
  // public static boolean allianceColorRed;
  public enum CommandSelector {
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
        "Five Ball Auto Red",
        new FiveBallAutoRed(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "Five Ball Auto Blue",
        new FiveBallAutoBlue(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "Replacement Auto",
        new ReplacementAuto(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "Replacement and Shoot",
        new ReplaceandShoot(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.setDefaultOption(
    //     "Five Ball Auto New",
    //     new FiveBallAuto(
    //         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.addOption("Drive Forward", new DriveForwardDistance(m_driveTrain, m_fieldSim,
    // 3));
    m_autoChooser.addOption("Do Nothing", new InstantCommand());
    m_autoChooser.addOption(
        "Two Ball Auto",
        new TwoBallAuto(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "Two Ball Auto Defense",
        new TwoBallAutoDefense(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.addOption(
    //     "Two Ball Auto Lower Hub",
    //     new TwoBallAutoLowerHub(
    //         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.addOption(
    //     "Five Ball Auto Vision",
    //     new FiveBallAutoLongShotVision(
    //         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));

    m_autoChooser.addOption(
        "Ball Trajectory",
        new CargoTrajectoryRameseteCommand(m_driveTrain, m_vision)
            .alongWith(new RunIntake(m_intake)));
    // m_autoChooser.addOption(
    //     "Three Ball Auto",
    //     new ThreeBallAuto(
    //         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.addOption(
    //     "Three Ball Auto Lower Hub",
    //     new ThreeBallAutoLowerHub(
    //         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.addOption(
    //     "Four Ball Auto",
    //     new FourBallAuto(
    //         m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    // m_autoChooser.addOption("Test Path", new TestPath(m_driveTrain, m_fieldSim));
    m_autoChooser.addOption(
        "Four Ball Auto",
        new FourBallAuto(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    m_autoChooser.addOption(
        "One Ball Auto Defense",
        new OneBallAutoDefense(
            m_driveTrain, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));
    SmartDashboard.putData("Selected Auto", m_autoChooser);

    DataLogManager.start(); // DATALOGGING
    indexerRPMLog = new DoubleLogEntry(m_logger, "/indexerRPM");
    flywheelRPMLog = new DoubleLogEntry(m_logger, "/flywheelRPM");
    kickerRPMLog = new DoubleLogEntry(m_logger, "/kickerRPM");
    poseLog = new StringLogEntry(m_logger, "/pose");
    // SmartDashboard.putData(
    //     "Auto Trajectory",
    //     new CargoTrajectoryRameseteCommand(m_driveTrain, m_vision)
    //         .alongWith(new RunIntake(m_intake, m_indexer)));

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
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 90));

    // leftButtons[0].whileHeld(
    //     new CargoTrajectoryRameseteCommand(m_driveTrain, m_vision)
    //         .alongWith(new RunIntake(m_intake, m_indexer)));
    // rightButtons[0].whenReleased(new IntakePiston(m_intake, false));

    rightButtons[0].whileHeld(
        new AlignToCargo(m_driveTrain, m_vision, leftJoystick::getY, rightJoystick::getX));
    // rightButtons[1].whileHeld(
    //     new AlignToLaunchpad(m_driveTrain, m_vision, leftJoystick::getY, rightJoystick::getX));

    // rightButtons[1].whileHeld(new RunIndexer(m_intake, m_indexer, m_flywheel, true));

    xBoxLeftTrigger =
        new Button(
            () -> xBoxController.getLeftTriggerAxis() > 0.2); // getTrigger());// getRawAxis(2));
    xBoxRightTrigger = new Button(() -> xBoxController.getRightTriggerAxis() > 0.2);

    xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_flywheel, m_vision, () -> m_flywheel.tarmacShot));
    xBoxButtons[1].whileHeld(
        new SetRpmSetpoint(m_flywheel, m_vision, () -> m_flywheel.launchpadShot));
    xBoxButtons[3].whileHeld(
        new SetRpmSetpoint(
            m_flywheel,
            m_vision,
            () ->
                ShotSelecter.interpolateRPM(
                    m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT))));
    // xBoxButtons[3].whileHeld(
    //     new SetRpmSetpoint(
    //         m_flywheel,
    //         m_vision,
    //         () ->
    //             ShotSelecter.bestShot(
    //                 m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT))));

    xBoxButtons[6].whenPressed(new ToggleTurretControlMode(m_turret));

    xBoxButtons[7].whenPressed(new ToggleTurretLock(m_turret));

    xBoxPOVButtons[2].whileHeld(new ReverseIntakeIndexer(m_intake, m_indexer));
    xBoxPOVButtons[0].whileHeld(new RunIndexer(m_intake, m_indexer, m_flywheel, false));
    xBoxLeftTrigger.whileHeld(new RunIntake(m_intake));
    xBoxLeftTrigger.whileHeld(new RunOnlyIndexer(m_indexer));
    xBoxRightTrigger.whileHeld(new RunIndexer(m_intake, m_indexer, m_flywheel, true));
    // xBoxRightTrigger.whileHeld(new LogShootingInfo(m_flywheel, m_indexer));

    xBoxButtons[2].whenPressed(new EngageHighClimb(m_climber));

    xBoxButtons[9].whenPressed(new SetClimbState(m_climber, true));
    xBoxButtons[9].whenPressed(
        new SetTurretAbsoluteSetpointDegrees(m_turret, 0)
            .andThen(new SetTurretControlMode(m_turret, false)));

    // xBoxButtons[6].whenPressed(new SetClimbState(m_climber, true));
    // xBoxButtons[7].whenPressed(new SetClimbState(m_climber, false));
    // leftButtons[0].cancelWhenPressed(m_driveTrain.getPostAutoCommand());
    // TODO Try this if the above does not work
    // leftButtons[0].cancelWhenPressed(m_driveTrain.getCurrentCommand());
    SmartDashboard.putData(new ResetOdometry(m_driveTrain, m_fieldSim, new Pose2d()));
    SmartDashboard.putData(new SetGoalLEDState(m_vision, false));
  }

  public void initializeSubsystems() {
    m_driveTrain.setDefaultCommand(
        new SetArcadeDrive(m_driveTrain, leftJoystick::getY, rightJoystick::getX));
    m_led.setDefaultCommand(new GetSubsystemStates(m_led, m_intake, m_flywheel, m_climber));
    m_climber.setDefaultCommand(
        new SetClimberOutput(m_climber, () -> xBoxController.getRawAxis(5)));
    // m_indexer.setDefaultCommand(
    //     new ColorSensor(m_indexer, m_controls, m_intake, m_flywheel, () ->
    // xBoxRightTrigger.get()));
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

  public void updateFieldSim() {
    m_fieldSim.periodic();
  }

  public void updateSmartDashboard() {
    m_driveTrain.updateSmartDashboard();
  }

  public void updateDriveTrainPeriodic() {
    m_driveTrain.periodicRunnable();
  }

  public void updateVisionPeriodic() {
    m_vision.periodicRunnable();
  }

  public void disabledInit() {
    m_driveTrain.setDriveTrainNeutralMode(DriveTrainNeutralMode.COAST);
    m_driveTrain.setMotorTankDrive(0, 0);
    m_driveTrain.setPostAutoCommand(null);
    // if (m_climber.getHighClimbPistonPosition() == Value.kForward) {
    //   m_climber.setClimberNeutralMode(NeutralMode.Coast);
    // }
    m_turret.setTurretNeutralMode(NeutralMode.Coast);
    m_vision.setVisionPoseEstimation(true);
    xBoxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    xBoxController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
  }

  public void disabledPeriodic() {
    m_vision.setVisionPoseEstimation(true);
    // SmartDashboard.putNumber(
    //     "Closest RPM",
    //
    // ShotSelecter.bestShot(m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT)));
  }

  public void teleopInit() {
    m_driveTrain.setDriveTrainNeutralMode(DriveTrainNeutralMode.BRAKE);
    m_climber.setClimberNeutralMode(NeutralMode.Brake);
    m_climber.setHoldPosition(m_climber.getElevatorClimbPosition());
    if (m_driveTrain.getPostAutoCommand() != null) {
      m_driveTrain.getPostAutoCommand().schedule(true);
    }
    m_intake.setIntakePiston(false);
    m_vision.setVisionPoseEstimation(true);
    m_flywheel.setRPM(0);
    m_fieldSim.clearAutoTrajectory();
  }

  public void teleopPeriodic() {
    m_vision.setVisionPoseEstimation(true);
    indexerRPMLog.append(m_indexer.getIndexerOutput());
    flywheelRPMLog.append(m_flywheel.getRPM(0));
    kickerRPMLog.append(m_indexer.getKickerOutput());
    poseLog.append(m_driveTrain.getRobotPoseMeters().toString());
  }

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
    m_vision.setVisionPoseEstimation(false);
  }

  public void autonomousPeriodic() {
    m_vision.setVisionPoseEstimation(false);
  }

  public void simulationInit() {
    m_fieldSim.initSim();
  }

  public void simulationPeriodic() {
    m_fieldSim.simulationPeriodic();
  }
}
