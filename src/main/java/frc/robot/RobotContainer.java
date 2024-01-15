// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcTurn;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DefaultArmMovement;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveDistanceSmartMotion;
import frc.robot.commands.DriveDistanceTrapazoidal;
import frc.robot.commands.FullSpeed;
import frc.robot.commands.MoveToPostition;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.SwitchSquareDrive;
import frc.robot.commands.TestAuto;
import frc.robot.commands.ThirdSpeed;
import frc.robot.commands.TurnDegrees;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  GenericHID m_leftjoystick = new GenericHID(0);
  GenericHID m_rightjoystick = new GenericHID(1);
  GenericHID m_armjoystick = new GenericHID(2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    m_driverSubsystem.setDefaultCommand(new MoveToPostition(m_driverSubsystem));
    // m_driverSubsystem.setDefaultCommand(
    //   new DefaultDrive(
    //     m_driverSubsystem,
    //     () -> -m_leftjoystick.getRawAxis(1),
    //     () -> -m_rightjoystick.getRawAxis(1)));
    m_armSubsystem.setDefaultCommand(new DefaultArmMovement(m_armSubsystem, () -> -m_armjoystick.getRawAxis(1)));
      
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
    new JoystickButton(m_leftjoystick, 12).onTrue(new DriveDistance(m_driverSubsystem,12, 0.15));
    new JoystickButton(m_leftjoystick, 7).onTrue(new DriveDistanceTrapazoidal(m_driverSubsystem,120, 0.5));
    new JoystickButton(m_leftjoystick, 11).onTrue(new TurnDegrees(m_driverSubsystem,90));
    new JoystickButton(m_rightjoystick, 2).onTrue(new ArcTurn(m_driverSubsystem, 18 , 180));
    new JoystickButton(m_leftjoystick, 8).onTrue(new DriveDistanceSmartMotion(m_driverSubsystem,120, 0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 

   return new TestAuto(m_driverSubsystem);
  }

  public void periodic(){
    SmartDashboard.putData("DriveSubsystem", m_driverSubsystem);
  }
}
