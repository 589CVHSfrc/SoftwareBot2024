// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistanceTrapazoidal extends CommandBase {
  double rampDistance;
  /** Creates a new drivedistance. */
  DriveSubsystem m_drive;
  double m_distance;
  double m_startPosition;
  double m_speed;
  public DriveDistanceTrapazoidal(DriveSubsystem drive, double distance, double speed) {
    m_drive = drive;
    m_distance = distance;
    addRequirements(m_drive);
    m_speed = speed;
    rampDistance = 40;
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
    double distanceTravled = m_drive.getDistanceleftmotor() - m_startPosition;
    double distanceRemaining = m_distance - distanceTravled;
    if (distanceRemaining <= rampDistance) {
      m_drive.drive(Math.max((distanceRemaining/rampDistance) * m_speed, 0.07),
       Math.max((distanceRemaining/rampDistance) * m_speed, 0.07));
    }
    else if (distanceTravled < rampDistance){
      m_drive.drive(Math.max((distanceTravled/rampDistance)* m_speed, 0.125), 
      Math.max((distanceTravled/rampDistance)* m_speed, 0.125));
    }
    else{
      m_drive.drive(m_speed, m_speed);
    }
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
