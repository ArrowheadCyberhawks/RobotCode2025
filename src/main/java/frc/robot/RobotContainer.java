// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import lib.frc706.cyberlib.XboxControllerWrapper;
import lib.frc706.cyberlib.commands.ToPointCommand;
import lib.frc706.cyberlib.commands.ToTagCommand;
import lib.frc706.cyberlib.commands.TrackPointCommand;
import lib.frc706.cyberlib.commands.XboxDriveCommand;

import lib.frc706.cyberlib.subsystems.*;
import frc.robot.subsystems.*;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.GrabberConstants.GrabberPosition;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.PID.PointTrack;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.ManualPivotCommand;
import frc.robot.Constants.ReefPoint;
import frc.robot.Constants.SwerveConstants;

import java.io.File;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  public final SwerveSubsystem swerveSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final PhotonCameraWrapper cam0, cam1, cam2, cam3, cam4, cam5, cam6;


  private final Elevator elevator;
  private final Grabber grabber;
  private final Intake intake;

  
  private Command teleopCommand;
  
  private Trigger[] nearTriggers, nearLeftTriggers, nearRightTriggers, farTriggers, farLeftTriggers, farRightTriggers;
  
  private final XboxControllerWrapper driverController;
  private final CommandXboxController manipulatorController;
  private final CommandGenericHID keypadHID;
  private final AutoCommandManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // set up controllers
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
    // set up keypad
    keypadHID = new CommandGenericHID(IOConstants.kKeypadPort);

    // set up swerve + photonvision
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    cam0 = new PhotonCameraWrapper("cam0", new Transform3d(new Translation3d(Inches.of(-4.75), Inches.of(15.75), Inches.of(22.875)), new Rotation3d(0,0, Math.PI/2))); // left side 
    cam1 = new PhotonCameraWrapper("cam1", new Transform3d(new Translation3d(Inches.of(-1), Inches.of(14.5), Inches.of(22.875)), new Rotation3d(0,0,0))); // left front
    cam2 = new PhotonCameraWrapper("cam2", new Transform3d(new Translation3d(Inches.of(-8), Inches.of(14.5), Inches.of(22.875)), new Rotation3d(0,0,Math.PI))); // left back
    cam3 = new PhotonCameraWrapper("cam3", new Transform3d(new Translation3d(Inches.of(9.25), Inches.of(14.5), Inches.of(27.5)), new Rotation3d(0,0,Math.PI)));
    cam4 = new PhotonCameraWrapper("cam4", new Transform3d(new Translation3d(Inches.of(-1), Inches.of(-15.5), Inches.of(27.125)), new Rotation3d(0,0,0))); //right front
    cam5 = new PhotonCameraWrapper("cam5", new Transform3d(new Translation3d(Inches.of(-4.75), Inches.of(-15.75), Inches.of(27.125)), new Rotation3d(0,0,-Math.PI/2))); //right side
    cam6 = new PhotonCameraWrapper("cam6", new Transform3d(new Translation3d(Inches.of(-8), Inches.of(-13.75), Inches.of(27.125)), new Rotation3d(0,0,-Math.PI))); // right back 
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, SwerveConstants.kMaxVelTele, PID.PathPlanner.kTranslationPIDConstants, PID.PathPlanner.kThetaPIDConstants, cam1, cam2, cam4, cam5);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // set up limelight
    //limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false, false, "limelight","limelight-three");
    limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false, false);
    
    // set up subsystems
    elevator = new Elevator();
    grabber = new Grabber();
    intake = new Intake();

    // commands and stuff
    autoManager = new AutoCommandManager(swerveSubsystem, intake, elevator, grabber);
    
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
    //Drive Controller
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
    
    driverController.x().whileTrue(new TrackPointCommand(swerveSubsystem,
        PointTrack.kThetaController,
        () -> ReefPoint.kCenter.getPose(),
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightTriggerAxis(),
        SwerveConstants.kMaxVelTele, SwerveConstants.kMaxAngularVelTele)
    );

    driverController.b().whileTrue(new ToPointCommand(swerveSubsystem,
      PointTrack.kXController, PointTrack.kYController, PointTrack.kThetaController,
      () -> Utils.getClosestReefPoint(swerveSubsystem.getPose()).getPose(),
      PointTrack.desiredDistance,
      SwerveConstants.kMaxVelAuto,
      SwerveConstants.kMaxAngularVelAuto)
    );

    driverController.y().whileTrue(new ToPointCommand(() -> ReefPoint.kFarLeftL.getPose(), PointTrack.desiredDistance));
  
    //TODO change to different keybind
    driverController.start().whileTrue(new ToTagCommand(swerveSubsystem, "limelight"));
    

    //X-Keys Lightboard
    nearTriggers = new Trigger[]{keypadHID.button(22), keypadHID.button(23)};
    nearLeftTriggers = new Trigger[]{keypadHID.button(13), keypadHID.button(17)};
    nearRightTriggers = new Trigger[]{keypadHID.button(20), keypadHID.button(16)};
    farTriggers = new Trigger[]{keypadHID.button(3), keypadHID.button(2)};
    farLeftTriggers = new Trigger[]{keypadHID.button(5), keypadHID.button(9)};
    farRightTriggers = new Trigger[]{keypadHID.button(12), keypadHID.button(8)};

    poseButtons(nearTriggers, "Near");
    poseButtons(nearLeftTriggers, "NearLeft");
    poseButtons(nearRightTriggers, "NearRight");
    poseButtons(farTriggers, "Far");
    poseButtons(farLeftTriggers, "FarLeft");
    poseButtons(farRightTriggers, "FarRight");

    elevatorButtons(19, "LO");
    elevatorButtons(7, "HI");
    elevatorButtons(6, "L4");
    elevatorButtons(10, "L3");
    elevatorButtons(14, "L2");
    elevatorButtons(18, "L1");

    keypadHID.button(15).onTrue(new SequentialCommandGroup(
      elevator.DEF(),
      new WaitCommand(0.9),
      grabber.setPivotPositionCommand(GrabberPosition.DOWN)
    ));
    
    keypadHID.button(11).onTrue(new ParallelCommandGroup(
      grabber.runGrabberCommand(-1).withTimeout(2),
      elevator.PICK()
    ));

    //Manipulator Controller
    manipulatorController.leftStick().whileTrue(new ManualElevatorCommand(elevator,
      () -> -manipulatorController.getLeftY())
    );

    manipulatorController.rightStick().whileTrue(new ManualPivotCommand(grabber, () -> -manipulatorController.getRightY()));
    manipulatorController.a().onTrue(grabber.setPivotPositionCommand(GrabberPosition.OUT)); //TODO debug
    manipulatorController.y().onTrue(grabber.setPivotPositionCommand(GrabberPosition.UP)); //TODO debug

    /*keypadHID.button(19).onTrue(elevator.setLevelCommand(ElevatorLevel.LO).alongWith(grabber.setPivotPositionCommand(GrabberPosition.UP)));
    keypadHID.button(7).onTrue(elevator.setLevelCommand(ElevatorLevel.HI).alongWith(grabber.setPivotPositionCommand(GrabberPosition.UP)));
    keypadHID.button(6).onTrue(elevator.setLevelCommand(ElevatorLevel.L4).alongWith(grabber.setPivotPositionCommand(GrabberPosition.UP)));
    keypadHID.button(10).onTrue(elevator.setLevelCommand(ElevatorLevel.L3).alongWith(grabber.setPivotPositionCommand(GrabberPosition.UP)));
    keypadHID.button(14).onTrue(elevator.setLevelCommand(ElevatorLevel.L2).alongWith(grabber.setPivotPositionCommand(GrabberPosition.UP)));
    keypadHID.button(18).onTrue(elevator.setLevelCommand(ElevatorLevel.L1).alongWith(grabber.setPivotPositionCommand(GrabberPosition.UP))); //12.75 inces 
    */

    

    //turn on intake
    //keypadHID.button(11).onTrue(intake.toggleIntakeCommand(true));


    //temp, not as many things
    manipulatorController.pov(0).whileTrue(grabber.runGrabberCommand(-0.25)); //was 0.25
    manipulatorController.pov(180).whileTrue(grabber.runGrabberCommand(0.25));

    //keypadHID.button(1).onTrue(grabber.setpiv));
    //keypadHID.button(15).onTrue(grabber.setPivotPositionCommand(GrabberPosition.UP));
    //keypadHID.button(1).onTrue(grabber.runPivotCommand(0.4));

    //keypadHID.button(4).onTrue(grabber.setPivotAngleCommand(new Rotation2d(0)));
   // manipulatorController.leftBumper().onTrue(intake.runIntakeCommand(1)); //retract // ignore brokeafied code 
    //manipulatorController.rightBumper().onTrue(intake.runIntakeCommand(1)); //retract
//MONKEY CODE 
    keypadHID.button(11).whileTrue(intake.runIntakeCommand(1));

   // keypadHID.button(4).onTrue(intake.runRetractCommand(1)); //maybe fix code   MAYBE TEST LATER 
  //  keypadHID.button(15).onTrue(intake.runRetractCommand(-1));  MAYBE TEST LATER

  }

  private void poseButtons(Trigger[] triggers, String name) {
    triggers[0].whileTrue(new ToPointCommand(() -> ReefPoint.valueOf("k" + name + "L").getPose(),  PointTrack.desiredDistance));
    triggers[1].whileTrue(new ToPointCommand(() -> ReefPoint.valueOf("k" + name + "R").getPose(),  PointTrack.desiredDistance));
    triggers[0].and(triggers[1]).whileTrue(new ToPointCommand(() -> ReefPoint.valueOf("k" + name + "C").getPose(), PointTrack.desiredDistance));
  }

  private void elevatorButtons(int buttonNum, String name) {
    keypadHID.button(buttonNum).onTrue(
      elevator.setLevelCommand(ElevatorLevel.valueOf(name))
      .andThen(new WaitCommand(0.3))
      .andThen(grabber.setPivotPositionCommand(GrabberPosition.UP))
    );
  }
  

  public Command getTeleopCommand() {
    return teleopCommand;
  }

  public Command getAutonomousCommand() {
    Command autoCommand = autoManager.getAutoManagerSelected();

    SmartDashboard.putString("Auto Selected", autoManager.getAutoManagerSelected().toString());
    return autoCommand;
  } 
}
