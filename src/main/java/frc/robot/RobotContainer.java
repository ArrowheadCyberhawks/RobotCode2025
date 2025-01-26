// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import lib.frc706.cyberlib.commands.XboxDriveCommand;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.commands.*;
import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  public final SwerveSubsystem swerveSubsystem;

  private Command teleopCommand;
  
  
  private final CommandXboxController driverController;
  private final CommandXboxController manipulatorController;
  private AutoCommandManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (DriverStation.isJoystickConnected(OperatorConstants.kDriverControllerPortBT)) {
      driverController = new CommandXboxController(OperatorConstants.kDriverControllerPortBT);
    } else {
      driverController = new CommandXboxController(OperatorConstants.kDriverControllerPortUSB);
    }
    if (DriverStation.isJoystickConnected(OperatorConstants.kManipulatorControllerPortBT)) {
      manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPortBT);
    } else {
      manipulatorController = new CommandXboxController(OperatorConstants.kManipulatorControllerPortUSB);
    }
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, OperatorConstants.kMaxVelTele,
        SwerveConstants.pathFollowerConfig);
    swerveSubsystem.swerveDrive.getGyro().setInverted(true);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    autoManager = new AutoCommandManager(swerveSubsystem);

    teleopCommand = new XboxDriveCommand(driverController,
        swerveSubsystem,
        () -> true,
        OperatorConstants.kDriverControllerDeadband,
        OperatorConstants.kMaxVelTele,
        OperatorConstants.kMaxAccelTele,
        OperatorConstants.kMaxAngularVelTele,
        OperatorConstants.kMaxAngularAccelTele).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        swerveSubsystem.setDefaultCommand(getTeleopCommand());

    configureBindings();
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
    driverController.a()
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.zeroHeading();
          swerveSubsystem.swerveDrive.synchronizeModuleEncoders();
          System.out.println("zeroing");
        })) // reset gyro to 0 degrees when A is pressed
        .debounce(2) // check if A is pressed for 2 seconds
        .onTrue(swerveSubsystem.runOnce(() -> {
          swerveSubsystem.recenter();
          System.out.println("resetting robot pose");
        })); // zero heading and reset position to (0,0) if A is pressed for 2 seconds

    driverController.b().whileTrue(new ToSpeakerCommand(swerveSubsystem)
      );
    driverController.rightBumper().whileTrue(new TrackSpeakerCommand(swerveSubsystem, 
      () -> -driverController.getLeftX(),
      () -> -driverController.getLeftY(), () -> driverController.getRightTriggerAxis() 
    ));
  } 

  public Command getTeleopCommand() {
    swerveSubsystem.swerveDrive.setHeadingCorrection(false);
    return teleopCommand;
  }

    /*public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    SmartDashboard.putString("Auto Selected", "Test Auto");
    return new PathPlannerAuto("Test Auto");
  }*/

    public Command getAutonomousCommand() {
      Command autoCommand = autoManager.getAutoManagerSelected();
      /*if(autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@66af20") || autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@12a209c") || autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@93b025") || autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@505305")){ //MStage4, MStage3, MAmp3, M2
        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(1.37, 5.56), new Rotation2d())); 
      } else if(autoManager.getAutoManagerSelected().toString().equals("com.pathplanner.lib.commands.PathPlannerAuto@7ddf94")){ //Amp2
        swerveSubsystem.setPose(new Rotation2d(), new Pose2d(new Translation2d(0.76, 6.78), new Rotation2d())); 
      } */

      SmartDashboard.putString("Auto Selected", autoManager.getAutoManagerSelected().toString());
      return autoCommand;
    } 
  

}
