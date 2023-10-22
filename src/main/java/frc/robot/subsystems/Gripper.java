// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  /** Creates a new Gripper. */

  private double angle;


  private TalonFX angu = new TalonFX(frc.robot.Constants.Gripper.angularMotor);
  private CANSparkMax paize = new CANSparkMax(frc.robot.Constants.Gripper.piezeMotor, MotorType.kBrushless);

  public Gripper() {

  angle = 0.0;

  angu.configFactoryDefault();
  angu.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);
  angu.setNeutralMode(NeutralMode.Brake);

  angu.setInverted(TalonFXInvertType.Clockwise);
  angu.setSensorPhase(false);

  angu.config_kP(0,0.0, 30);
  angu.config_kI(0,0.1, 30);
  angu.config_kI(0,0.0, 30);
  angu.config_kF(0,0.0, 30);

  angu.configNominalOutputForward(0);
  angu.configNominalOutputReverse(0);
  angu.configPeakOutputForward(1);
  angu.configPeakOutputReverse(-1);


  paize.restoreFactoryDefaults();
  paize.setIdleMode(IdleMode.kBrake);
  paize.setInverted(false);


  }

  public void speed(double speed){

    paize.set(speed);
  }


  public double getAngle(){
    return angu.getSelectedSensorPosition();

  }

  public void setArmSpeed(double value){
    angu.set(ControlMode.PercentOutput, value);
  }

  public void setAngle(double value){
    angle = value;
  }

  public void runPID(){
    angu.set(TalonFXControlMode.Position, angle);
  }

  @Override
  public void periodic() {
  //  angu.set(TalonFXControlMode.Position, angle);
  SmartDashboard.putNumber("encoderFalcon", getAngle());



  }
}
