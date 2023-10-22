// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ChargeStation extends CommandBase {
  private final Drivetrain drivetrain;

  private static double MAX_VELOCITY = Drivetrain.MAX_VELOCITY_METERS_PER_SECOND;
  private static double MAX_OMEGA = Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  private double kP = 0.2;

  /* private final  double kP = 0.01,
                        kF = 0.08; */

  /** Creates a new DriveCommand. */
  public ChargeStation(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              -0.4 * MAX_VELOCITY,
              0,
              0.0,
              drivetrain.getGyroscopeRotation()
      )
    );
    Timer.delay(2.0); */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    drivetrain.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
              //modifyAxis(-0.4) * MAX_VELOCITY,
              modifyAxis(maxValue((drivetrain.getPitch()-0)*kP, 0.4)) * MAX_VELOCITY,
              0.0,
              0.0,
              drivetrain.getGyroscopeRotation()
      )
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.1*MAX_OMEGA));
    Timer.delay(0.03);
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(drivetrain.getPitch()) <= 13.5;//FIXME
  }

  private static double modifyAxis(double value) {
    // Deadband
    //value = deadband(value, 0.11);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private double maxValue(double value, double max){
    if (value > max)
      return max;
    else if (value < -max)
      return -max;
    else
      return value;
  }
}
