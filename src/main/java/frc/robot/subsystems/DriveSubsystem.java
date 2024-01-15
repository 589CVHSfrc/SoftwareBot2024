
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivePIDConstants;


public class DriveSubsystem extends SubsystemBase {
  CANSparkMax m_leftMotor;
  CANSparkMax m_rightMotor;
  RelativeEncoder m_rightEncoder;
  RelativeEncoder m_leftEncoder;
  SparkMaxPIDController m_rightMotorPIDController, m_leftMotorPIDController;
  DifferentialDrive m_drive;
  boolean m_squaredInput;
  double SpeedMultiplier;
  double m_desiredPosition;
  private double kP,kI,kD;
  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftMotor = new CANSparkMax(41, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(12, MotorType.kBrushless);

    m_rightMotorPIDController = m_rightMotor.getPIDController();
    m_leftMotorPIDController = m_leftMotor.getPIDController();

    m_rightEncoder = m_rightMotor.getEncoder();
    m_leftEncoder = m_leftMotor.getEncoder();

    double PositionConversionFactor = (Math.PI * 6)/10.71;
    m_leftEncoder.setPositionConversionFactor(PositionConversionFactor);
    m_rightEncoder.setPositionConversionFactor(PositionConversionFactor);

    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_squaredInput = false;
    SpeedMultiplier = 1.0;

    m_desiredPosition = 0;
  
    m_leftMotor.setInverted(false);
    m_rightMotor.setInverted(true);

    m_leftMotorPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr,
        DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartMotionSlot);

    m_rightMotorPIDController.setSmartMotionMaxVelocity(DrivePIDConstants.maxVel, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setSmartMotionMinOutputVelocity(DrivePIDConstants.minVel, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setSmartMotionMaxAccel(DrivePIDConstants.maxAcc, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setSmartMotionAllowedClosedLoopError(DrivePIDConstants.allowedErr,
        DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, DrivePIDConstants.smartMotionSlot);


    m_rightMotorPIDController.setP(DrivePIDConstants.kPsv, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setI(DrivePIDConstants.kIsv, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setD(DrivePIDConstants.kDsv, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setIZone(DrivePIDConstants.kIzsv, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setFF(DrivePIDConstants.kFFsv, DrivePIDConstants.smartMotionSlot);

    m_leftMotorPIDController.setP(DrivePIDConstants.kPsv, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setI(DrivePIDConstants.kIsv, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setD(DrivePIDConstants.kDsv, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setIZone(DrivePIDConstants.kIzsv, DrivePIDConstants.smartMotionSlot);
    m_leftMotorPIDController.setFF(DrivePIDConstants.kFFsv, DrivePIDConstants.smartMotionSlot);

    SmartDashboard.putNumber("Desired Distance", 0);
    kP = DrivePIDConstants.kP;
    kI = DrivePIDConstants.kI;
    kD = DrivePIDConstants.kD;
    SmartDashboard.putNumber("P value", DrivePIDConstants.kP);
    SmartDashboard.putNumber("I value", DrivePIDConstants.kI);
    SmartDashboard.putNumber("D value", DrivePIDConstants.kD);
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

  public void PIDDistance(double distance) {
    m_leftMotorPIDController.setReference(distance, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setReference(distance, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
  }
  public void PIDDesiredDistance() {
    m_leftMotorPIDController.setReference(m_desiredPosition, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
    m_rightMotorPIDController.setReference(m_desiredPosition, CANSparkMax.ControlType.kSmartMotion, DrivePIDConstants.smartMotionSlot);
    SmartDashboard.putNumber("Moving to", m_desiredPosition);
  }

  public void safteyModeOff(){
    m_drive.setSafetyEnabled(false);
  }
  public void safteyModeOn(){
    m_drive.setSafetyEnabled(true);
  }

  @Override 
  public void periodic() {
      m_desiredPosition = SmartDashboard.getNumber("Desired Distance", 0);
      // System.out.print("=================");
      // System.out.print(m_desiredPosition);
    //   System.out.print(DesiredPostition);
    // if(DesiredPostition != m_desiredPosition){
    //     m_desiredPosition = DesiredPostition;
    //     System.out.print("----------------");
    //   }
    // PIDDistance(DesiredPostition);

    double P = SmartDashboard.getNumber("P value", 0);
    double I = SmartDashboard.getNumber("I value", 0);
    double D = SmartDashboard.getNumber("D value", 0);

    //System.out.println(P);

    if(P != kP){
      m_leftMotorPIDController.setP(P);
      m_rightMotorPIDController.setP(P);
      kP = P;
      System.out.println("~~~~~~~~~~ P value changed to: " + P + " ~~~~~~~~~~");
    }

    if(I != kI){
      m_leftMotorPIDController.setI(I);
      m_rightMotorPIDController.setI(I);
      kI = I;
    }

    if(D != kD){
      m_leftMotorPIDController.setD(D);
      m_rightMotorPIDController.setD(D);
      kD = D;
    }
  
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left motor position", getDistanceleftmotor());
    //SmartDashboard.put("DriveSubsystem",this);
    
  }
}
