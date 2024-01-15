// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  DriveSubsystem m_drive;
  public TestAuto(DriveSubsystem drive) {
    m_drive = drive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveDistance(m_drive, 36, .15), new TurnDegrees(m_drive, 90), new DriveDistance(m_drive, 46, 0.15),
    new TurnDegrees(m_drive, 0), 
    new DriveDistance(m_drive, 28, 0.15), 
    new TurnDegrees(m_drive, 58), 
    new DriveDistance(m_drive, 88, 0.15), 
    new ArcTurn(m_drive, 18, 210), new DriveDistance(m_drive, 48, 0.15),
    new TurnDegrees(m_drive, 50), new DriveDistance(m_drive, 81, 0.15), new TurnDegrees(m_drive, 65), 
    new DriveDistance(m_drive, 50, 0.15));
  }
}
