// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  DifferentialDrive m_drive;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftMotor = new CANSparkMax(11, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(12, MotorType.kBrushless);
    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  
    m_leftMotor.setInverted(true);
  }

  public void drive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
