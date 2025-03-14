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
import frc.robot.commands.SetSuperstructureCommand;
// import frc.robot.subsystems.Intake;
import frc.robot.Constants.ReefPoint;
import frc.robot.Constants.SwerveConstants;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final PhotonCameraWrapper cam1, cam2, cam3, cam4, cam5, cam6;


  private final Elevator elevator;
  final Grabber grabber;
  private final Climber climber;
  // private final Intake intake;

  
  private Command teleopCommand;
  
  private Trigger[] nearTriggers, nearLeftTriggers, nearRightTriggers, farTriggers, farLeftTriggers, farRightTriggers;
  
  private final XboxControllerWrapper driverController;
  private final CommandXboxController manipulatorController;
  private final CommandGenericHID keypadHID;
  private final AutoCommandManager autoManager;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // set up subsystems
    // these should be down below swervesubsystem but i need the elevator
    elevator = new Elevator();
    grabber = new Grabber();
    climber = new Climber();

    // set up controllers
    if (DriverStation.isJoystickConnected(IOConstants.kDriverControllerPortBT)) {
      driverController = new XboxControllerWrapper(IOConstants.kDriverControllerPortBT, IOConstants.kDriverControllerDeadband, 0.15) {
        @Override
        public double interpolate(double value) {
        return value * MathUtil.interpolate(0.15, 1, getRightTriggerAxis() - elevator.getHeight().in(Meters) / 1.7);
    }
      };
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
    // cam0 = new PhotonCameraWrapper("cam0", new Transform3d(new Translation3d(Inches.of(15.75), Inches.of(4.75), Inches.of(22.875)), new Rotation3d(0, 0, 0))); // DOES NOT EXIST
    cam1 = new PhotonCameraWrapper("cam1", new Transform3d(new Translation3d(Inches.of(13.625), Inches.of(6.75), Inches.of(30.5)), new Rotation3d(Units.degreesToRadians(-8.3),0, Math.PI/2))); // front left
    cam2 = new PhotonCameraWrapper("cam2", new Transform3d(new Translation3d(Inches.of(12.625), Inches.of(4.75), Inches.of(30.5)), new Rotation3d(0,0, 0))); // front forwards
    cam3 = new PhotonCameraWrapper("cam3", new Transform3d(new Translation3d(Inches.of(13.625), Inches.of(2.75), Inches.of(30.5)), new Rotation3d(Units.degreesToRadians(-8.3),0, -Math.PI/2))); // front right
    cam4 = new PhotonCameraWrapper("cam4", new Transform3d(new Translation3d(Inches.of(-13.625), Inches.of(2.75), Inches.of(30)), new Rotation3d(Units.degreesToRadians(8.3),0, -Math.PI/2))); // rear right
    cam5 = new PhotonCameraWrapper("cam5", new Transform3d(new Translation3d(Inches.of(-12.625), Inches.of(4.75), Inches.of(30)), new Rotation3d(0,0, Math.PI))); // rear backwards
    cam6 = new PhotonCameraWrapper("cam6", new Transform3d(new Translation3d(Inches.of(-13.625), Inches.of(6.75), Inches.of(30)), new Rotation3d(Units.degreesToRadians(8.3),0, Math.PI/2))); // rear left
    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, SwerveConstants.kMaxVelTele.in(MetersPerSecond), PID.PathPlanner.kTranslationPIDConstants, PID.PathPlanner.kThetaPIDConstants, cam1, cam2, cam3, cam4, cam5, cam6);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // set up limelight
    //limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false, false, "limelight","limelight-three");
    limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false, false);
    
    // intake = new Intake();

    // commands and stuff
    autoManager = new AutoCommandManager(swerveSubsystem, elevator, grabber);
    
    teleopCommand = new XboxDriveCommand(driverController,
        swerveSubsystem,
        () -> true,
        IOConstants.kDriverControllerDeadband,
        SwerveConstants.kMaxVelTele.in(MetersPerSecond),
        SwerveConstants.kMaxAccelTele.in(MetersPerSecondPerSecond),
        SwerveConstants.kMaxAngularVelTele.in(RadiansPerSecond),
        SwerveConstants.kMaxAngularAccelTele.in(RadiansPerSecondPerSecond))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
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
        SwerveConstants.kMaxVelTele.in(MetersPerSecond), SwerveConstants.kMaxAngularVelTele.in(RadiansPerSecond))
    );

    driverController.b().whileTrue(new ToPointCommand(swerveSubsystem,() -> ReefPoint.kFarLeftR.getPose())
    );

    driverController.y().whileTrue(AutoCommandManager.pathfindCommand(ReefPoint.kFarLeftL.getPose())
    );
  
    //TODO change to different keybind
    driverController.start().whileTrue(new ToTagCommand(swerveSubsystem, "limelight"));
    
    driverController.povUp().whileTrue(climber.runClimbCommand(() -> 0.8));
    driverController.povDown().whileTrue(climber.runClimbCommand(() -> -0.2));

    //X-KEYS LIGHTBOARD
    nearTriggers = new Trigger[]{keypadHID.button(22), keypadHID.button(23)};
    nearLeftTriggers = new Trigger[]{keypadHID.button(13), keypadHID.button(17)};
    nearRightTriggers = new Trigger[]{keypadHID.button(20), keypadHID.button(16)};
    farTriggers = new Trigger[]{keypadHID.button(3), keypadHID.button(2)};
    farLeftTriggers = new Trigger[]{keypadHID.button(5), keypadHID.button(9)};
    farRightTriggers = new Trigger[]{keypadHID.button(12), keypadHID.button(8)};

    //Move to Pose
    poseButtons(nearTriggers, "Near");
    poseButtons(nearLeftTriggers, "NearLeft");
    poseButtons(nearRightTriggers, "NearRight");
    poseButtons(farTriggers, "Far");
    poseButtons(farLeftTriggers, "FarLeft");
    poseButtons(farRightTriggers, "FarRight");

    //Elevator Presets
    elevatorButtons(19, "LO");
    keypadHID.button(7).whileTrue(new SetSuperstructureCommand(grabber, elevator, GrabberPosition.HI::getAngle, ElevatorLevel.HI::getHeight));
    keypadHID.button(6).whileTrue(new SetSuperstructureCommand(grabber, elevator, GrabberPosition.L4::getAngle, ElevatorLevel.L4::getHeight));
    elevatorButtons(10, "L3");
    elevatorButtons(14, "L2");
    keypadHID.button(18).whileTrue(new SetSuperstructureCommand(grabber, elevator, GrabberPosition.L1::getAngle, ElevatorLevel.L1::getHeight));
    keypadHID.button(21).whileTrue(new SetSuperstructureCommand(grabber, elevator, GrabberPosition.HUMAN::getAngle, ElevatorLevel.HUMAN::getHeight));


    //MANIPULATOR CONTROLLER

    //Manual Elevator control
    manipulatorController.leftStick().whileTrue(new ManualElevatorCommand(elevator, () -> -manipulatorController.getLeftY()/50));
    manipulatorController.rightStick().whileTrue(new ManualPivotCommand(grabber, () -> -manipulatorController.getRightY()/8));

    //Pivot
    manipulatorController.a().onTrue(grabber.setPivotPositionCommand(GrabberPosition.OUT)); //TODO debug
    manipulatorController.y().onTrue(grabber.setPivotPositionCommand(GrabberPosition.HI)); //TODO debug

    //Intake/Outtake
    manipulatorController.pov(0).whileTrue(grabber.intakeCommand());
    manipulatorController.pov(180).whileTrue(grabber.outtakeCommand());
    manipulatorController.povRight().onTrue(new InstantCommand(() -> grabber.resetPivotTarget()).ignoringDisable(true));

    //free button 1, 4, 6, 15

    
    // keypadHID.button(1).whileTrue(intake.runIntakeCommand(0.1));
    // keypadHID.button(4).whileTrue(intake.runIntakeCommand(-0.1));
    
    //IM FIRED

    // keypadHID.button(6).whileTrue(intake.runRetractCommand(0.1));
    // keypadHID.button(15).whileTrue(intake.runExtendCommand(0.1));
  
  }

  private void poseButtons(Trigger[] triggers, String name) {
    triggers[0].whileTrue(new ToPointCommand(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "L").getPose()));
    triggers[1].whileTrue(new ToPointCommand(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "R").getPose()));
    triggers[0].and(triggers[1]).whileTrue(new ToPointCommand(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "C").getPose()));
  }

  private void elevatorButtons(int buttonNum, String name) {
    keypadHID.button(buttonNum).onTrue(new SetSuperstructureCommand(grabber, elevator, GrabberPosition.PLACE::getAngle, ElevatorLevel.valueOf(name)::getHeight));
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
