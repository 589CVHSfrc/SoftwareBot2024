// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  RelativeEncoder m_rightEncoder;
  RelativeEncoder m_leftEncoder;
  DifferentialDrive m_drive;
  boolean m_squaredInput;
  double SpeedMultiplier;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftMotor = new CANSparkMax(11, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(12, MotorType.kBrushless);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_leftEncoder = m_leftMotor.getEncoder();

    double PositionConversionFactor = (Math.PI * 6)/10.71;
    m_leftEncoder.setPositionConversionFactor(PositionConversionFactor);
    m_rightEncoder.setPositionConversionFactor(PositionConversionFactor);

    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_squaredInput = false;
    SpeedMultiplier = 1.0;
  
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);
  }

  public double getDistanceleftmotor(){
    return m_leftEncoder.getPosition();
  }

  public double getDistancerightmotor(){
    return m_rightEncoder.getPosition();
  }

  public void resetEncoders(){
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    
  }

  public void drive(double left, double right) {
   
   // m_drive.tankDrive(left, right);
   
   m_drive.tankDrive(SpeedMultiplier * left, SpeedMultiplier * right, m_squaredInput);
    
  }

  public void squaredSpeed(){
    m_squaredInput=!m_squaredInput;
  }

  public void SetSpeed(double speed)
  {
    if(speed <= 1) {
      SpeedMultiplier = speed;
    }
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left motor position", getDistanceleftmotor());
    
  }
}
