// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.DriveCommand;
import frc.robot.commands.PositionAprilTag;

import frc.robot.subsystems.Drivetrain;


import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final PathPlannerFiles pFiles = new PathPlannerFiles();
  private final Drivetrain drivetrain = new Drivetrain();
  private final LimeLightServer lServer = new LimeLightServer();
  //private final Elevator elevator = new Elevator();
  //private final Gripper gripper = new Gripper();
  //private final Intake intake = new Intake();
  //private final Telescopic_Arm telescopic_Arm = new Telescopic_Arm();

  SendableChooser<String> auto = new SendableChooser<>();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private CommandXboxController m_secondController =
      new CommandXboxController(Constants.OperatorConstants.kSecondControllerPort);

  
  HashMap<String, Command> eventMap = new HashMap<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    drivetrain.setDefaultCommand(new DriveCommand(
            drivetrain,
            () -> -modifyAxis(m_driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    auto.addOption("Barrier", "barrier");
    auto.addOption("MID", "mid");
    auto.addOption("Courner", "courner");
    auto.setDefaultOption("Default", "default");

    SmartDashboard.putData("auto", auto);

    


    // Configure the trigger bindings
    configureBindings();
    setEventMaps();

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.14);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().toggleOnTrue(new PositionAprilTag(drivetrain, lServer));

  



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return new SequentialCommandGroup(followTrajectoryCommandWithEvents(pFiles.rectLine), new PrintCommand("Path finish :)!"));

    if (auto.getSelected() == "barrier"){
      return followTrajectoryCommandWithEvents(pFiles.barrier);
    }
    else if(auto.getSelected() == "mid"){
      //return followTrajectoryCommandWithEvents(pFiles.chargue);
      //return new ChargeStation(drivetrain); 
      return new SequentialCommandGroup(followTrajectoryCommandWithEvents(pFiles.chargue));//, new ChargeStation(drivetrain));
    }
    else if(auto.getSelected() == "courner"){
      return followTrajectoryCommandWithEvents(pFiles.courner);
    }
    else{
      return null;
    }
  }

  public Command followTrajectoryCommandWithEvents(PathPlannerTrajectory traj) {
    PIDController theta = new PIDController(Constants.Autonomous.kPThetaController, 0, 0);
    //PathPlannerTrajectory trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());
    theta.enableContinuousInput(-Math.PI, Math.PI);
    
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first and unique path you run during auto
          if(true){//isFirstPath){
            //drivetrain.resetPose(traj.getInitialHolonomicPose());
            var transformedState = PathPlannerTrajectory.transformStateForAlliance(traj.getInitialState(), DriverStation.getAlliance());
            drivetrain.resetPose(new Pose2d(transformedState.poseMeters.getTranslation(), transformedState.holonomicRotation));
          }
        }),
        new FollowPathWithEvents(new PPSwerveControllerCommand(
            //Funcion agregada para forzar de manera unica el modo espejo en la trajectoria, usando useAllianceColor dentro de esta funcion no se ejecuta como debería
            traj,//trajectory,
            drivetrain::getPose, // Pose supplier
            drivetrain.getKinematics(), // SwerveDriveKinematics
            new PIDController(Constants.Autonomous.kPXController, 0.0, 0.0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.Autonomous.kPYController, 0.0, 0.0), // Y controller (usually the same values as X controller)
            theta, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            drivetrain::driveAutonomous, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            drivetrain // Requires this drive subsystem
        ), 
        traj.getMarkers(),
        eventMap
    ));
  }

  public void setEventMaps(){
    eventMap.put("mark1", new PrintCommand("Marca 1 pass"));
    eventMap.put("mark2", new PrintCommand("Marca 2 pass"));
    eventMap.put("mark3", new PrintCommand("Marca 3 pass"));
    //eventMap.put("intakeON", null);
    /* eventMap.put("gripperIn", gripper.getPiece().andThen(gripper.neutralMode()));
    eventMap.put("gripperOut", gripper.outPiece().andThen(gripper.neutralMode()));
    eventMap.put("gripperOff", gripper.neutralMode()); */
  }

}