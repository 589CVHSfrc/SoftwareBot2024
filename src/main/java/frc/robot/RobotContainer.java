// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FullSpeed;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.SwitchSquareDrive;
import frc.robot.commands.ThirdSpeed;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driverSubsystem = new DriveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  GenericHID m_leftjoystick = new GenericHID(0);
  GenericHID m_rightjoystick = new GenericHID(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driverSubsystem.setDefaultCommand(
      new DefaultDrive(
        m_driverSubsystem,
        () -> -m_leftjoystick.getRawAxis(1),
        () -> -m_rightjoystick.getRawAxis(1)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings(){
    new JoystickButton(m_leftjoystick, 5).onTrue(new SwitchSquareDrive(m_driverSubsystem));
    new JoystickButton(m_leftjoystick, 2).onTrue(new FullSpeed(m_driverSubsystem));
    new JoystickButton(m_leftjoystick, 1).onTrue(new ThirdSpeed(m_driverSubsystem));
    new JoystickButton(m_rightjoystick, 1).whileTrue(new AutoDrive(m_driverSubsystem));
    new JoystickButton(m_rightjoystick, 6).whileTrue(new ResetEncoders(m_driverSubsystem));
    new JoystickButton(m_leftjoystick, 12).onTrue(new DriveDistance(m_driverSubsystem,12));
    new JoystickButton(m_leftjoystick, 7).onTrue(new DriveDistance(m_driverSubsystem,60));
    new JoystickButton(m_leftjoystick, 11).onTrue(new TurnDegrees(m_driverSubsystem,90));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 

   return null;
  }
}
