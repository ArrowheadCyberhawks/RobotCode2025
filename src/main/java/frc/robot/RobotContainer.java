// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import lib.frc706.cyberlib.XboxControllerWrapper;
import lib.frc706.cyberlib.commands.ToPointCommand;
import lib.frc706.cyberlib.commands.ToTagCommand;
import lib.frc706.cyberlib.commands.TrackPointCommand;
import lib.frc706.cyberlib.commands.controller.XboxDriveCommand;
import lib.frc706.cyberlib.subsystems.*;
import frc.robot.subsystems.*;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.auto.DriveToPose;
import frc.robot.auto.AlignToReef;

import frc.robot.commands.LEDCommand;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.ManualPivotCommand;
import frc.robot.commands.SetSuperstructureCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IOConstants;
import frc.robot.constants.Constants.PID;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.constants.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.constants.Constants.GrabberConstants.GrabberPosition;
import frc.robot.constants.Constants.PID.PointTrack;

import java.io.File;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

  //subsystems
  public static SwerveSubsystem swerveSubsystem;
  private final Elevator elevator;
  public final Pivot pivot;
  private final Grabber grabber;
  private final Climber climber;
  private final LEDSubsystem ledSubsystem;

  private final LimelightSubsystem limelightSubsystem;
  private final PhotonCameraWrapper cam0, cam1, cam2, cam3, cam4, cam5, cam6;
 
  private Command teleopCommand;
  
  private Trigger[] nearTriggers, nearLeftTriggers, nearRightTriggers, farTriggers, farLeftTriggers, farRightTriggers;
  
  private final XboxControllerWrapper driverController;
  private final CommandXboxController manipulatorController;
  private final CommandGenericHID keypadHID;
  private final AutoCommandManager autoManager;
  private final AlignToReef alignmentCommandFactory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    //Init Subsystems
    elevator = new Elevator();
    pivot = new Pivot();
    grabber = new Grabber();
    climber = new Climber();
    ledSubsystem = new LEDSubsystem();

    //Init controllers
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
    cam0 = new PhotonCameraWrapper(Constants.CameraConstants.cam0.name, Constants.CameraConstants.cam0.offset);
    cam1 = new PhotonCameraWrapper(Constants.CameraConstants.cam1.name, Constants.CameraConstants.cam1.offset);

    cam2 = new PhotonCameraWrapper("cam2", new Transform3d(new Translation3d(Inches.of(12.625), Inches.of(4.75), Inches.of(30.5)), new Rotation3d(0,0, 0))); // front forwards
    cam3 = new PhotonCameraWrapper("cam3", new Transform3d(new Translation3d(Inches.of(13.625), Inches.of(2.75), Inches.of(30.5)), new Rotation3d(Units.degreesToRadians(-8.3),0, -Math.PI/2))); // front right
    cam4 = new PhotonCameraWrapper("cam4", new Transform3d(new Translation3d(Inches.of(-13.625), Inches.of(2.75), Inches.of(29)), new Rotation3d(Units.degreesToRadians(8.3),0, -Math.PI/2))); // rear right
    cam5 = new PhotonCameraWrapper("cam5", new Transform3d(new Translation3d(Inches.of(-12.625), Inches.of(4.75), Inches.of(29.75)), new Rotation3d(0,0, Math.PI))); // rear backwards
    cam6 = new PhotonCameraWrapper("cam6", new Transform3d(new Translation3d(Inches.of(-13.625), Inches.of(6.75), Inches.of(30.25)), new Rotation3d(Units.degreesToRadians(8.3),0, Math.PI/2))); // rear left

    //cam0 = new PhotonCameraWrapper("cam0", new Transform3d(new Translation3d(Inches.of(-1.75), Inches.of(-3.1875), Inches.of(5)), new Rotation3d(0, Units.degreesToRadians(-20), 0))); // reef
    //cam1 = new PhotonCameraWrapper("cam1", new Transform3d(new Translation3d(Inches.of(13.625), Inches.of(6.75), Inches.of(30.5)), new Rotation3d(Units.degreesToRadians(-8.3),0, Math.PI/2))); // front left

    cam0.photonPoseEstimator.setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, SwerveConstants.kMaxVelTele.in(MetersPerSecond), PID.PathPlanner.kTranslationPIDConstants, PID.PathPlanner.kThetaPIDConstants, cam0, cam1, cam2, cam3, cam4, cam5, cam6);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // set up limelight
    limelightSubsystem = new LimelightSubsystem(swerveSubsystem, false, false);

    // commands and stuff
    autoManager = new AutoCommandManager(swerveSubsystem, elevator, pivot, grabber, climber);
    alignmentCommandFactory = new AlignToReef(swerveSubsystem);
    
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
    ledSubsystem.setDefaultCommand(new LEDCommand(ledSubsystem));
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
    // elevatorButtons(18, "LO");
    keypadHID.button(7).onTrue(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.HI::getAngle, ElevatorLevel.HI::getHeight));
    keypadHID.button(6).onTrue(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.L4::getAngle, ElevatorLevel.L4::getHeight));
    
    keypadHID.button(18).onTrue(new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
		.withTimeout(0.5)
		.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ZERO::getAngle, ElevatorLevel.CLEAR::getHeight))
    	.withTimeout(2)
		.onlyIf(() -> elevator.getHeight().in(Meters) < ElevatorLevel.LO.getHeight())
    	.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ZERO::getAngle, ElevatorLevel.LO::getHeight)));

    elevatorButtons(10, "L3");
    // elevatorButtons(14, "L2");

    //L2
    keypadHID.button(14).onTrue(new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
		.withTimeout(0.5)
		.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.PLACE::getAngle, ElevatorLevel.CLEAR::getHeight))
    	.withTimeout(2)
		.onlyIf(() -> elevator.getHeight().in(Meters) < ElevatorLevel.CLEAR.getHeight())
    	.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.PLACE::getAngle, ElevatorLevel.L2::getHeight)));
    // keypadHID.button(18).whileTrue(new SetSuperstructureCommand(grabber, elevator, GrabberPosition.L1::getAngle, ElevatorLevel.L1::getHeight));
    keypadHID.button(21).onTrue(new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
		.withTimeout(0.5)
		.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.PLACE::getAngle, ElevatorLevel.CLEAR::getHeight))
    	.withTimeout(2)
		.onlyIf(() -> elevator.getHeight().in(Meters) < ElevatorLevel.CLEAR.getHeight())
    	.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.HUMAN::getAngle, ElevatorLevel.HUMAN::getHeight)));

    keypadHID.button(19).onTrue(new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
		.withTimeout(0.5)
		.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ALGPICK::getAngle, ElevatorLevel.CLEAR::getHeight))
    	.withTimeout(2)
		.onlyIf(() -> elevator.getHeight().in(Meters) < ElevatorLevel.CLEAR.getHeight())
    	.andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ALGPICK::getAngle, ElevatorLevel.LO::getHeight)));

    keypadHID.button(15).onTrue(new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
     .withTimeout(0.5)
     .andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ALGREEF::getAngle, ElevatorLevel.ALG2::getHeight))
       .withTimeout(2)
     .onlyIf(() -> elevator.getHeight().in(Meters) < ElevatorLevel.CLEAR.getHeight())
       .andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ALGREEF::getAngle, ElevatorLevel.ALG2::getHeight)));

    keypadHID.button(11).onTrue(new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
      .withTimeout(0.5)
      .andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ALGREEF::getAngle, ElevatorLevel.ALG3::getHeight))
        .withTimeout(2)
      .onlyIf(() -> elevator.getHeight().in(Meters) < ElevatorLevel.CLEAR.getHeight())
        .andThen(new SetSuperstructureCommand(pivot, elevator, GrabberPosition.ALGREEF::getAngle, ElevatorLevel.ALG3::getHeight)));


  


    //MANIPULATOR CONTROLLER

    //Manual Elevator control
    manipulatorController.leftStick().whileTrue(new ManualElevatorCommand(elevator, () -> -manipulatorController.getLeftY()/50));
    manipulatorController.rightStick().whileTrue(new ManualPivotCommand(pivot, () -> -manipulatorController.getRightY()/8));

    //Intake/Outtake
    manipulatorController.pov(0).whileTrue(grabber.intakeCommand());
    manipulatorController.pov(180).whileTrue(grabber.outtakeCommand());
    manipulatorController.povRight().onTrue(new InstantCommand(() -> pivot.resetPivotTarget()).ignoringDisable(true));

    manipulatorController.a().debounce(2).onTrue(new InstantCommand(() -> {
      pivot.resetPivotAngle(new Rotation2d());
      System.out.println("resetting pivot angle");
    }).ignoringDisable(true));

    // manipulatorController.b().whileTrue(AutoCommandManager.pathfindThenPIDCommand(new Pose2d(12.07, 1.305, new Rotation2d())));
    manipulatorController.b().whileTrue(new DriveToPose(swerveSubsystem, () -> ReefPoint.kFarLeftL.getPose()));
    manipulatorController.x().whileTrue(alignmentCommandFactory.generateCommand(ReefPoint.kFarLeftL));

    //manipulatorController.b().whileTrue(AutoCommandManager.pathfindThenPIDCommand(ReefPoint.kFarLeftL.getPose()));
    //free button 1, 4, 6, 15

    
    // keypadHID.button(1).whileTrue(intake.runIntakeCommand(0.1));
    // keypadHID.button(4).whileTrue(intake.runIntakeCommand(-0.1));
    
    //IM FIRED

    // keypadHID.button(6).whileTrue(intake.runRetractCommand(0.1));
    // keypadHID.button(15).whileTrue(intake.runExtendCommand(0.1));
  
  }

  private void poseButtons(Trigger[] triggers, String name) {
    triggers[0].whileTrue(new DriveToPose(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "L").getPose()));
    triggers[0].whileTrue(new DriveToPose(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "R").getPose()));
    triggers[0].and(triggers[1]).whileTrue(new DriveToPose(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "C").getPose()));

    /*
    triggers[0].whileTrue(new ToPointCommand(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "L").getPose()));
    triggers[1].whileTrue(new ToPointCommand(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "R").getPose()));
    triggers[0].and(triggers[1]).whileTrue(new ToPointCommand(swerveSubsystem, () -> ReefPoint.valueOf("k" + name + "C").getPose()));
    */
    // triggers[0].whileTrue(AutoCommandManager.pathfindToReefCommand(name + "L"));
    // triggers[1].whileTrue(AutoCommandManager.pathfindThenPIDCommand(ReefPoint.valueOf("k" + name + "R").getPose()));
    // triggers[0].and(triggers[1]).whileTrue(AutoCommandManager.pathfindThenPIDCommand(ReefPoint.valueOf("k" + name + "C").getPose()));
  }

  private void elevatorButtons(int buttonNum, String name) {
    keypadHID.button(buttonNum).onTrue(
      // new SetSuperstructureCommand(grabber, elevator, grabber::getPivotAngle, ElevatorLevel.L4::getHeight).withTimeout(0.5)
      (new SetSuperstructureCommand(pivot, elevator, GrabberPosition.PLACE::getAngle, ElevatorLevel.valueOf(name)::getHeight)));
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
