// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TurnDegrees extends CommandBase {
  /** Creates a new TurnDegrees. */
  DriveSubsystem m_drive;
  double m_degrees;
  double m_distanceNeeded, m_startPositionRight, m_startPositionLeft;
  public TurnDegrees(DriveSubsystem drive, double degrees) {
    m_drive = drive;
    m_degrees = degrees;
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startPositionLeft = m_drive.getDistanceleftmotor();
    m_startPositionRight = m_drive.getDistancerightmotor();
    m_distanceNeeded = ((19.5*Math.PI)/360)*m_degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_degrees > 0){
      m_drive.drive(.125,-.125);
    }
    else{
      m_drive.drive(-.125,.125);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_degrees > 0){
      if(m_drive.getDistanceleftmotor() - m_startPositionLeft >= m_distanceNeeded){
        return true;
      }
    }
    else{
      if(m_drive.getDistancerightmotor() - m_startPositionRight >= m_distanceNeeded) {
        return true;
      }
    }
    
    return false;
  }
}
