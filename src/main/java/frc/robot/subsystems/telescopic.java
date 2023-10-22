// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Telescopic;

public class telescopic extends SubsystemBase {
  
  private CANSparkMax m1 = new CANSparkMax(Telescopic.TELESCOPIC_MOTOR, MotorType.kBrushless);

  SparkMaxPIDController pidController;
  RelativeEncoder encoder;

  private double position;

  /** Creates a new telescopic. */
  public telescopic() {

  position = 0.0;  

  m1.restoreFactoryDefaults();
  m1.setInverted(false); //FIXME
  m1.setIdleMode(IdleMode.kBrake);

  pidController = m1.getPIDController();

  encoder = m1.getEncoder();

  pidController.setP(0);
  pidController.setI(0);
  pidController.setD(0);

  encoder.setPosition(0);


  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("encoderTelescopic", encoder.getPosition());
   // m1.getPIDController().setReference(position, ControlType.kPosition);
  }
}
