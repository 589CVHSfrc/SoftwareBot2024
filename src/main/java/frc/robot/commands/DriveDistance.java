// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {
  /** Creates a new drivedistance. */
  DriveSubsystem m_drive;
  double m_distance;
  double m_startPosition;
  public DriveDistance(DriveSubsystem drive, double distance) {
    m_drive = drive;
    m_distance = distance;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPosition = m_drive.getDistanceleftmotor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(0.15, 0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_drive.getDistanceleftmotor()-m_startPosition>=m_distance){
      return true;
    }
    return false;
  }
}
