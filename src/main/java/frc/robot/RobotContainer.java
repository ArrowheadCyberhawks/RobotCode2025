// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import lib.frc706.cyberlib.commands.XboxDriveCommand;
import lib.frc706.cyberlib.subsystems.LimelightSubsystem;
import lib.frc706.cyberlib.subsystems.PhotonCameraWrapper;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.*;
import java.io.File;

import edu.wpi.first.math.geometry.*;
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
  private final LimelightSubsystem limelightSubsystem;
  private final PhotonCameraWrapper cam0;
  
  private Command teleopCommand;
  
  
  private final CommandXboxController driverController;
  private final CommandXboxController manipulatorController;
  private AutoCommandManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (DriverStation.isJoystickConnected(IOConstants.kDriverControllerPortBT)) {
      driverController = new CommandXboxController(IOConstants.kDriverControllerPortBT);
    } else {
      driverController = new CommandXboxController(IOConstants.kDriverControllerPortUSB);
    }
    if (DriverStation.isJoystickConnected(IOConstants.kManipulatorControllerPortBT)) {
      manipulatorController = new CommandXboxController(IOConstants.kManipulatorControllerPortBT);
    } else {
      manipulatorController = new CommandXboxController(IOConstants.kManipulatorControllerPortUSB);
    }
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    
    cam0 = new PhotonCameraWrapper("cam0", new Transform3d(new Translation3d(Inches.of(9), Inches.of(-3), Inches.of(8)), new Rotation3d(0,0,0)));
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, SwerveConstants.kMaxVelTele, PID.kTranslationPIDConstants, PID.kThetaPIDConstants, cam0);
    limelightSubsystem = new LimelightSubsystem(swerveSubsystem, "limelight-threeg","limelight-three");
    //limelightSubsystem = new LimelightSubsystem(swerveSubsystem);
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
    
    driverController.x().whileTrue(new TrackReefCommand(swerveSubsystem, 
        () -> -driverController.getLeftX(),
        () -> -driverController.getLeftY() 
    ));
    driverController.b().whileTrue(new ToReefCommand(swerveSubsystem)
      );
    /*driverController.rightBumper().whileTrue(new TrackReefCommand(swerveSubsystem, 
      () -> -driverController.getLeftX(),
      () -> -driverController.getLeftY(), () -> driverController.getRightTriggerAxis() 
    ));*/
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
