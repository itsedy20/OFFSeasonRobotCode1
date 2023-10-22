// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.arm;

public class Arm extends SubsystemBase {
  private CANSparkMax m1 = new CANSparkMax(arm.armM1, MotorType.kBrushless);
  private CANSparkMax m2 = new CANSparkMax(arm.armM2, MotorType.kBrushless);

  private SparkMaxPIDController mp, mp2;

  private RelativeEncoder en1, en2;
  
  private double position;

  /** Creates a new Arm. */
  public Arm() {

  position = 0.0;
  
  mp = m1.getPIDController();
  mp2 = m2.getPIDController();

  en1 = m1.getEncoder();
  en2 =  m2.getEncoder();

  m1.restoreFactoryDefaults();
  m1.setIdleMode(IdleMode.kBrake);
  m1.setInverted(false);
 
  mp.setP(0.0);
  mp.setI(0.0);
  mp.setD(0.0);

  m2.restoreFactoryDefaults();
  m2.setIdleMode(IdleMode.kBrake);
  m2.setInverted(false);
  m2.follow(m1);

  mp2.setP(0.0);
  mp2.setI(0.0);
  mp2.setD(0.0);


  en1.setPosition(0.0);
  en2.setPosition(0.0);

  }

  public CommandBase setHome() {
    
    return runOnce(
        () -> {
          position = 0;
        });
  }

 
  @Override
  public void periodic() {
  SmartDashboard.putNumber("arm",en1.getPosition());
  m1.getPIDController().setReference(position, ControlType.kPosition);

  }


 
}