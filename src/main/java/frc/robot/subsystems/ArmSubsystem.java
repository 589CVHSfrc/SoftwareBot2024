// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_armMotor = new CANSparkMax(11, MotorType.kBrushless);
  }

  public void defaultArmControl(double inputarm){

    if(Math.abs(inputarm) > 0.1 ) {
      m_armMotor.set(inputarm);
    } 
    else{
      m_armMotor.set(0);
    }
  }


  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
