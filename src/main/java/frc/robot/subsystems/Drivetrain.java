// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.FRC5705.HolaMundoXD;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
  public static final double MAX_VOLTAGE = 12;

  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeft= new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  Translation2d m_frontRight = new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0);
  Translation2d m_backLeft = new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  Translation2d m_backRight = new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
  
  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID, "rio");
  
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private final HolaMundoXD hol = new HolaMundoXD();
  
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  
  private final SwerveDriveOdometry m_odometry;
  public SwerveModule[] mSwerveMods;
  
  /** Creates a new Drivetrain. */
  public Drivetrain() {
        hol.hola();
        
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // There are 4 methods you can call to create your swerve modules.
        // The method you use depends on what motors you are using.
        //
        // Mk3SwerveModuleHelper.createFalcon500(...)
        //   Your module has two Falcon 500s on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createNeo(...)
        //   Your module has two NEOs on it. One for steering and one for driving.
        //
        // Mk3SwerveModuleHelper.createFalcon500Neo(...)
        //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
        //
        // Mk3SwerveModuleHelper.createNeoFalcon500(...)
        //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
        //
        // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

        // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
        // you MUST change it. If you do not, your code will crash on startup.
        // FIXME Setup motor configuration
        m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk4iSwerveModuleHelper.GearRatio.L3,
                // This is the ID of the drive motor
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                // This is the ID of the steer motor
                FRONT_LEFT_MODULE_STEER_MOTOR,
                // This is the ID of the steer encoder
                FRONT_LEFT_MODULE_STEER_ENCODER,
                // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        // We will do the same for the other modules
        m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L3,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );

        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(), new SwerveModulePosition[]{
                new SwerveModulePosition(0, new Rotation2d(m_frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(0, new Rotation2d(m_frontRightModule.getSteerAngle())),
                new SwerveModulePosition(0, new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModulePosition(0, new Rotation2d(m_backRightModule.getSteerAngle()))
        }, Autonomous.initialPose);

        //m_pigeon.configFactoryDefault();
        //zeroGyroscope();
        zeroGyroscopeToAngle(Autonomous.initialPose.getRotation().getDegrees());
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase resetGyro() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {

            });
      }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
  }

  /**
   * Reestablece el giroscopio a un ángulo específico. 
   * Útil para antes de autónomo
   * @param angle
   */
  public void zeroGyroscopeToAngle(double angle) {
        m_pigeon.setYaw(angle);
  }

  public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public double getPitch(){
        return m_pigeon.getPitch();
  }

  /*
   * Ingresa el objeto ChassisSpeeds para actualizar el rumbo actual.
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds; 
  }

  public void driveAutonomous(SwerveModuleState[] swerveModuleState) {
    ChassisSpeeds value = m_kinematics.toChassisSpeeds(swerveModuleState); 
    m_chassisSpeeds = value; 
  }

  public SwerveDriveKinematics getKinematics(){
        return m_kinematics;
  }

  public Pose2d getPose(){
        return m_odometry.getPoseMeters();
  }

  /**
   * Reset odometry
   * @param pose
   */
  public void resetPose(Pose2d pose){
        m_odometry.resetPosition(getGyroscopeRotation(), 
        new SwerveModulePosition[]{
                new SwerveModulePosition(m_frontLeftModule.getPosition(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_frontRightModule.getPosition(), new Rotation2d(m_frontRightModule.getSteerAngle())),
                new SwerveModulePosition(m_backLeftModule.getPosition(), new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_backRightModule.getPosition(), new Rotation2d(m_backRightModule.getSteerAngle()))
        }, pose);
  }

  @Override
  public void periodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

        m_odometry.update(getGyroscopeRotation(),
                new SwerveModulePosition[]{
                        new SwerveModulePosition(m_frontLeftModule.getPosition(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
                        new SwerveModulePosition(m_frontRightModule.getPosition(), new Rotation2d(m_frontRightModule.getSteerAngle())),
                        new SwerveModulePosition(m_backLeftModule.getPosition(), new Rotation2d(m_backLeftModule.getSteerAngle())),
                        new SwerveModulePosition(m_backRightModule.getPosition(), new Rotation2d(m_backRightModule.getSteerAngle()))
                }
        );
        
        SmartDashboard.putNumber("Position FL", m_frontLeftModule.getPosition());
        SmartDashboard.putNumber("Position FR", m_frontRightModule.getPosition());
        SmartDashboard.putNumber("Position BL", m_backLeftModule.getPosition());
        SmartDashboard.putNumber("Position BR", m_backRightModule.getPosition());
        SmartDashboard.putNumber("Pitch", getPitch());
  }
}
