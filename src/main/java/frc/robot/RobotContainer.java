// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.auto.OneBallAuto;
import frc.robot.commands.auto.ThreeBallAuto;
import frc.robot.commands.auto.TwoBallAuto;
import frc.robot.commands.driveTrain.SetArcadeDrive;
import frc.robot.simulation.FieldSim;

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

    static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
    static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
    static Joystick xBoxController = new Joystick(Constants.USB.xBoxController);

    public Button[] leftButtons = new Button[2];
    public Button[] rightButtons = new Button[2];
    public Button[] xBoxButtons = new Button[10];
    public Button[] xBoxPOVButtons = new Button[8];
    public Button xBoxLeftTrigger, xBoxRightTrigger;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initializeSubsystems();

        // Configure the button bindings
        configureButtonBindings();
    }

    public void initializeSubsystems() {
        m_driveTrain.setDefaultCommand(new SetArcadeDrive(m_driveTrain, leftJoystick::getY, rightJoystick::getX));
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new OneBallAuto(m_driveTrain, m_fieldSim);
    }

    public void robotPeriodic() {

    }

    public void disabledInit() {

    }

    public void disabledPeriodic() {

    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {

    }

    public void autonomousInit() {

    }

    public void autonomousPeriodic() {

    }

    public void simulationInit() {
        m_fieldSim.initSim();
    }

    public void simulationPeriodic() {
        m_fieldSim.simulationPeriodic();
    }
}
