// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
  }

  public static class Autonomous {
    //Iniciando mirando hacia el panel de drivers
    public static final Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));

    public static final double kPXController = 4.0;
    public static final double kPYController = 4.0;
    public static final double kPThetaController = 4.0;
  }

  public static class Drivetrain {
    public static final int DRIVETRAIN_PIGEON_ID = 60;
 
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19.75); // Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(14.12 ); // Measure and set wheelbase

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 18; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; // Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 20; // Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(88.67578125000001); // Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 13; // Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; // Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(21.795703125000003); // Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 15; // Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 21; // Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(254.87929687500005); // Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 16; // Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 23; // Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(206.71875); // Measure and set back right steer offset
  }

 
 public static class arm {

  public static final int armM1 = 50;
  public static final int armM2 = 49;
 }

  public static class Telescopic {
  
    public static final int TELESCOPIC_MOTOR = 42; //FIXME ID del motor del mecanismo telescopico
  }

  public static class Gripper {
   
   public static final int angularMotor = 55;
   public static final int piezeMotor = 51;
  }
}
