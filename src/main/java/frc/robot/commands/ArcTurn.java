// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcTurn extends CommandBase {
  /** Creates a new ArcTurn. */
  private DriveSubsystem m_driveSubsystem;
  private double m_initalLeftEncoderPosition;
  private double m_initalRightEncoderPosition;
  private double m_radius;
  private double m_degrees;
  private double m_motorRatio;
  public ArcTurn(DriveSubsystem driveSubsystem, double radius, double degrees) {
    m_driveSubsystem = driveSubsystem;
    addRequirements(m_driveSubsystem);
    m_initalLeftEncoderPosition = 0;
    m_initalRightEncoderPosition = 0;
    m_radius = radius;
    m_degrees = degrees;
    m_motorRatio = m_radius/(m_radius+Constants.PhysicalConstants.kRobotWidth);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initalLeftEncoderPosition = m_driveSubsystem.getDistanceleftmotor();
    m_initalRightEncoderPosition = m_driveSubsystem.getDistancerightmotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_degrees > 0) {
      m_driveSubsystem.drive(.25, m_motorRatio * .25 );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_degrees > 0){
      if(m_driveSubsystem.getDistancerightmotor()-m_initalRightEncoderPosition >= (2*Math.PI*m_radius / 360)*m_degrees){
        return true;
      }
    }
    return false;
  }
}
