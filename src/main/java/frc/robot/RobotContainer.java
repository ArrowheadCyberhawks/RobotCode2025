// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import lib.frc706.cyberlib.XboxControllerWrapper;
import lib.frc706.cyberlib.commands.ToPointCommand;
import lib.frc706.cyberlib.commands.TrackPointCommand;
import lib.frc706.cyberlib.commands.XboxDriveCommand;
import lib.frc706.cyberlib.subsystems.LimelightSubsystem;
import lib.frc706.cyberlib.subsystems.PhotonCameraWrapper;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants.PID.PointTrack;
import frc.robot.commands.*;
import java.io.File;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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
  private final LimelightSubsystem limelightSubsystem;
  private final PhotonCameraWrapper cam0, cam1, cam2;
  
  private Command teleopCommand;
  
  private Trigger nearTrigger, nearLeftTrigger, nearRightTrigger, farTrigger, farLeftTrigger, farRightTrigger, leftBranchTrigger, rightBranchTrigger, centerTrigger;
  
  private final XboxControllerWrapper driverController;
  private final CommandXboxController manipulatorController;
  private final CommandGenericHID keypadHID;
  private AutoCommandManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    if (DriverStation.isJoystickConnected(IOConstants.kDriverControllerPortBT)) {
      driverController = new XboxControllerWrapper(IOConstants.kDriverControllerPortBT, IOConstants.kDriverControllerDeadband, 0.15);
    } else {
      driverController = new XboxControllerWrapper(IOConstants.kDriverControllerPortUSB, IOConstants.kDriverControllerDeadband, 0.15);
    }
    if (DriverStation.isJoystickConnected(IOConstants.kManipulatorControllerPortBT)) {
      manipulatorController = new CommandXboxController(IOConstants.kManipulatorControllerPortBT);
    } else {
      manipulatorController = new CommandXboxController(IOConstants.kManipulatorControllerPortUSB);
    }
    keypadHID = new CommandGenericHID(IOConstants.kKeypadPort);

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
    cam0 = new PhotonCameraWrapper("cam0", new Transform3d(new Translation3d(Inches.of(0), Inches.of(12), Inches.of(4.75)), new Rotation3d(0,0,Math.PI/2)));
    cam1 = new PhotonCameraWrapper("cam1", new Transform3d(new Translation3d(Inches.of(9), Inches.of(-3), Inches.of(8)), new Rotation3d(0,0,0)));
    cam2 = new PhotonCameraWrapper("cam2", new Transform3d(new Translation3d(Inches.of(0), Inches.of(-12), Inches.of(4.75)), new Rotation3d(0,0,-Math.PI/2)));
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, SwerveConstants.kMaxVelTele, PID.PathPlanner.kTranslationPIDConstants, PID.PathPlanner.kThetaPIDConstants, cam0, cam1, cam2);
    // limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false, "limelight-threeg","limelight-three");
    limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    autoManager = new AutoCommandManager(swerveSubsystem);
    
    teleopCommand = new XboxDriveCommand(driverController,
        swerveSubsystem,
        () -> true,
        IOConstants.kDriverControllerDeadband,
        SwerveConstants.kMaxVelTele,
        SwerveConstants.kMaxAccelTele,
        SwerveConstants.kMaxAngularVelTele,
        SwerveConstants.kMaxAngularAccelTele).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
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
          swerveSubsystem.resetOdometry(new Pose2d());;
          System.out.println("resetting robot pose");
        })); // zero heading and reset position to (0,0) if A is pressed for 2 seconds
    
    driverController.x().whileTrue(new TrackPointCommand(swerveSubsystem, () -> Utils.getClosestReefPoint(swerveSubsystem.getPose()).getPose(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getLeftY(),
        () -> driverController.getRightTriggerAxis(),
        SwerveConstants.kMaxVelTele, SwerveConstants.kMaxAngularVelTele, true, PointTrack.kThetaController
      )
    );

    driverController.b().whileTrue(new ToPointCommand(swerveSubsystem,
      ()->Utils.getClosestReefPoint(swerveSubsystem.getPose()).getPose(),
      PointTrack.kXController, PointTrack.kYController, PointTrack.kThetaController, PointTrack.desiredDistance,
      SwerveConstants.kMaxVelAuto, SwerveConstants.kMaxAngularVelAuto)
    );
    /*driverController.rightBumper().whileTrue(new TrackReefCommand(swerveSubsystem, 
      () -> -driverController.getLeftX(),
      () -> -driverController.getLeftY(), () -> driverController.getRightTriggerAxis() 
    ));*/
    driverController.y().whileTrue(new ToPointCommand(swerveSubsystem, ()->ReefPoint.kFarLeftL.getPose(), PointTrack.kXController, PointTrack.kYController, PointTrack.kThetaController, PointTrack.desiredDistance, SwerveConstants.kMaxVelAuto, SwerveConstants.kMaxAngularVelAuto));
    
    nearTrigger = keypadHID.button(18);
    nearLeftTrigger = keypadHID.button(17);
    nearRightTrigger = keypadHID.button(19);
    
    farTrigger = keypadHID.button(15);
    farLeftTrigger = keypadHID.button(14);
    farRightTrigger = keypadHID.button(16);

    rightBranchTrigger = keypadHID.button(13);
    centerTrigger = keypadHID.button(12);
    leftBranchTrigger = keypadHID.button(11);

    poseButtons(nearTrigger, "Near");
    poseButtons(nearLeftTrigger, "NearLeft");
    poseButtons(nearRightTrigger, "NearRight");
    poseButtons(farTrigger, "Far");
    poseButtons(farLeftTrigger, "FarLeft");
    poseButtons(farRightTrigger, "FarRight");
  
  }

  private void poseButtons(Trigger trigger, String name) {
    trigger.and(leftBranchTrigger).whileTrue(new ToPointCommand(swerveSubsystem, ()->ReefPoint.valueOf("k" + name + "L").getPose(), PointTrack.kXController, PointTrack.kYController, PointTrack.kThetaController, PointTrack.desiredDistance, SwerveConstants.kMaxVelAuto, SwerveConstants.kMaxAngularVelAuto));
    trigger.and(centerTrigger).whileTrue(new ToPointCommand(swerveSubsystem, ()->ReefPoint.valueOf("k" + name + "C").getPose(), PointTrack.kXController, PointTrack.kYController, PointTrack.kThetaController, PointTrack.desiredDistance, SwerveConstants.kMaxVelAuto, SwerveConstants.kMaxAngularVelAuto));
    trigger.and(rightBranchTrigger).whileTrue(new ToPointCommand(swerveSubsystem,()->ReefPoint.valueOf("k" + name + "R").getPose(), PointTrack.kXController, PointTrack.kYController, PointTrack.kThetaController, PointTrack.desiredDistance, SwerveConstants.kMaxVelAuto, SwerveConstants.kMaxAngularVelAuto));
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

      SmartDashboard.putString("Auto Selected", autoManager.getAutoManagerSelected().toString());
      return autoCommand;
    } 
  

}
