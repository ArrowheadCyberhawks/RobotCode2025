// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import lib.frc706.cyberlib.XboxControllerWrapper;
import lib.frc706.cyberlib.commands.TrackPointCommand;
import lib.frc706.cyberlib.commands.controller.XboxDriveCommand;
import lib.frc706.cyberlib.subsystems.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.Superstructure.SuperStructureState;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.auto.AlignToReef;
import frc.robot.auto.DriveToPose;
import frc.robot.commands.ManualElevatorCommand;
import frc.robot.commands.ManualPivotCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.IOConstants;
import frc.robot.constants.Constants.PID;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.constants.Constants.SwerveConstants;
import frc.robot.constants.Constants.GrabberConstants.GrabberState;
import frc.robot.constants.Constants.PID.PointTrack;

import java.io.File;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// subsystems
	public static SwerveSubsystem swerveSubsystem;
	private final Elevator elevator;
	public final Arm pivot;
	private final Grabber grabber;
	private final Climber climber;

	private final Superstructure superstructure;
	private final LEDSubsystem ledSubsystem;

	private final PhotonCameraWrapper cam0, cam1;
	// private final PhotonCameraWrapper cam0, cam1, cam2, cam3, cam4, cam5, cam6;

	private Command teleopCommand;

	private Trigger[] nearTriggers, nearLeftTriggers, nearRightTriggers, farTriggers, farLeftTriggers, farRightTriggers;

	private final XboxControllerWrapper driverController;
	private final CommandXboxController manipulatorController;
	private final CommandGenericHID keypadHID;
	private final AutoCommandManager autoManager;

  //Automation CommandFactories
  private final AlignToReef alignmentCommandFactory;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
	
		// SmartDashboard.putNumber("DriveToPose/tp", 9.45);
		// SmartDashboard.putNumber("DriveToPose/ti", 0.62);
		// SmartDashboard.putNumber("DriveToPose/td", 0.02);


		WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

		// Init Subsystems
		elevator = new Elevator();
		pivot = new Arm();
		grabber = new Grabber();
		climber = new Climber();

		superstructure = new Superstructure(elevator, pivot, grabber);

		ledSubsystem = new LEDSubsystem();

		// Init controllers
		if (DriverStation.isJoystickConnected(IOConstants.kDriverControllerPortBT)) {
			driverController = new XboxControllerWrapper(IOConstants.kDriverControllerPortBT,
					IOConstants.kDriverControllerDeadband, 0.15) {
				@Override
				public double interpolate(double value) {
					return value * MathUtil.interpolate(0.15, 1, getRightTriggerAxis() - elevator.getHeight().in(Meters) / 1.7);
				}
			};
		} else {
			driverController = new XboxControllerWrapper(IOConstants.kDriverControllerPortUSB,
					IOConstants.kDriverControllerDeadband, 0.15) {
					@Override
				public double interpolate(double value) {
					return value * MathUtil.interpolate(0.15, 1, getRightTriggerAxis() - elevator.getHeight().in(Meters) / 1.7);
				}
			};
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
		cam1 = new PhotonCameraWrapper("cam2", Constants.CameraConstants.cam1.offset);

		// cam2 = new PhotonCameraWrapper("cam2", new Transform3d(new
		// Translation3d(Inches.of(12.625), Inches.of(4.75), Inches.of(30.5)), new
		// Rotation3d(0,0, 0))); // front forwards
		// cam3 = new PhotonCameraWrapper("cam3", new Transform3d(new
		// Translation3d(Inches.of(13.625), Inches.of(2.75), Inches.of(30.5)), new
		// Rotation3d(Units.degreesToRadians(-8.3),0, -Math.PI/2))); // front right
		// cam4 = new PhotonCameraWrapper("cam4", new Transform3d(new
		// Translation3d(Inches.of(-13.625), Inches.of(2.75), Inches.of(29)), new
		// Rotation3d(Units.degreesToRadians(8.3),0, -Math.PI/2))); // rear right
		// cam5 = new PhotonCameraWrapper("cam5", new Transform3d(new
		// Translation3d(Inches.of(-12.625), Inches.of(4.75), Inches.of(29.75)), new
		// Rotation3d(0,0, Math.PI))); // rear backwards
		// cam6 = new PhotonCameraWrapper("cam6", new Transform3d(new
		// Translation3d(Inches.of(-13.625), Inches.of(6.75), Inches.of(30.25)), new
		// Rotation3d(Units.degreesToRadians(8.3),0, Math.PI/2))); // rear left

		// cam0 = new PhotonCameraWrapper("cam0", new Transform3d(new
		// Translation3d(Inches.of(-1.75), Inches.of(-3.1875), Inches.of(5)), new
		// Rotation3d(0, Units.degreesToRadians(-20), 0))); // reef
		// cam1 = new PhotonCameraWrapper("cam1", new Transform3d(new
		// Translation3d(Inches.of(13.625), Inches.of(6.75), Inches.of(30.5)), new
		// Rotation3d(Units.degreesToRadians(-8.3),0, Math.PI/2))); // front left

		cam0.photonPoseEstimator.setPrimaryStrategy(PoseStrategy.LOWEST_AMBIGUITY);

		// 2 CAMERAS
		swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory, SwerveConstants.kMaxVelTele.in(MetersPerSecond),
				PID.PathPlanner.kTranslationPIDConstants, PID.PathPlanner.kThetaPIDConstants, cam0, cam1);

		// WITH ALL 7 CAMERAS
		// swerveSubsystem = new SwerveSubsystem(swerveJsonDirectory,
		// SwerveConstants.kMaxVelTele.in(MetersPerSecond),
		// PID.PathPlanner.kTranslationPIDConstants, PID.PathPlanner.kThetaPIDConstants,
		// cam0, cam1, cam2, cam3, cam4, cam5, cam6);
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		
		alignmentCommandFactory = new AlignToReef(swerveSubsystem, superstructure, grabber);

		// commands and stuff
		autoManager = new AutoCommandManager(swerveSubsystem, superstructure, grabber, climber);
		// alignmentCommandFactory = new AlignToReef(swerveSubsystem);

		//autoCycleCommandFactory = new AutoCycle(swerveSubsystem, superstructure, grabber);
		// alignmentCommandFactory = new AlignToReef(swerveSubsystem, elevator, pivot,
		// grabber);

		teleopCommand = new XboxDriveCommand(driverController,
				swerveSubsystem,
				driverController.rightBumper().negate()::getAsBoolean,
				IOConstants.kDriverControllerDeadband,
				SwerveConstants.kMaxVelTele.in(MetersPerSecond),
				SwerveConstants.kMaxAccelTele.in(MetersPerSecondPerSecond),
				SwerveConstants.kMaxAngularVelTele.in(RadiansPerSecond),
				SwerveConstants.kMaxAngularAccelTele.in(RadiansPerSecondPerSecond))
				.withInterruptBehavior(InterruptionBehavior.kCancelSelf);

		swerveSubsystem.setDefaultCommand(getTeleopCommand());

		ledSubsystem.setDefaultCommand(ledSubsystem.setLEDStateCommand(LEDState.DEFAULT));
		new Trigger(grabber::hasAlgae).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.ALGAE));
		new Trigger(grabber::hasCoral).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.CORAL));
		new Trigger(DriverStation::isAutonomous).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.AUTO));
		// new Trigger(() -> grabber.onReef() && grabber.hasCoral()).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.ONREEF));
		new Trigger(climber::isClimbing).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.CLIMB));
		new Trigger(climber::isClimbed).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.ISCLIMBED));
		new Trigger(grabber::isOutaking).whileTrue(ledSubsystem.setLEDStateCommand(LEDState.OUTTAKE));

		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// Drive Controller

		driverController.b()
				.onTrue(swerveSubsystem.runOnce(() -> {
					swerveSubsystem.zeroHeading();
					swerveSubsystem.swerveDrive.synchronizeModuleEncoders();
					System.out.println("zeroing");
				})) // reset gyro to 0 degrees when Y is pressed
				.debounce(2) // check if Y is pressed for 2 seconds
				.onTrue(swerveSubsystem.runOnce(() -> {
					swerveSubsystem.resetOdometry(new Pose2d());
					;
					System.out.println("resetting robot pose");
				})); // zero heading and reset position to (0,0) if Y is pressed for 2 seconds

		driverController.a().whileTrue(new TrackPointCommand(swerveSubsystem,
				PointTrack.kThetaController,
				() -> ReefPoint.kCenter.getPose(),
				() -> driverController.getLeftX(),
				() -> driverController.getLeftY(),
				() -> driverController.getRightTriggerAxis(),
				SwerveConstants.kMaxVelTele.in(MetersPerSecond), SwerveConstants.kMaxAngularVelTele.in(RadiansPerSecond)));

		driverController.leftBumper().whileTrue(
			new DriveToPose(swerveSubsystem, 
				Constants.FieldPosition.kBargeMiddle::getPose,
				() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red) ? driverController.getLeftX() : -driverController.getLeftX()
			)
		);

		driverController.leftTrigger().and(driverController.rightBumper()).whileTrue(
			new RunCommand(
				() -> swerveSubsystem.swerveDrive.drive(
					new ChassisSpeeds(
						-driverController.getHID().getLeftTriggerAxis() / 2,
						0,
						0
					)
				),
				swerveSubsystem
			)
			.finallyDo(swerveSubsystem::stopModules)
		);

		//Drive to Human Player Station
		// driverController.b().whileTrue();
		// driverController.x().whileTrue();
		//D-Pad- Move Translationally

    //Change Positioning Mode
    // driverController.back().onTrue();
    //driverController.start().onTrue(alignmentCommandFactory.switchMode());

		//TEMP
		//driverController.b().whileTrue(autoCycleCommandFactory.run());

    driverController.povUp().whileTrue(climber.runClimbCommand(() -> 0.8));
    driverController.povDown().whileTrue(climber.runClimbCommand(() -> -0.4));
    driverController.povLeft().onTrue(climber.climbOutCommand());
    driverController.povRight().onTrue(climber.climbInCommand());

    //Auto-Positioning
    driverController.leftTrigger(0.5).whileTrue(alignmentCommandFactory.generateCommand().alongWith(grabber.autoOutakeCommand()));
	//driverController.leftTrigger().whileTrue(grabber.autoOutakeCommand());

    // X-KEYS LIGHTBOARD
    // lightboard();
    nearTriggers = new Trigger[] {keypadHID.button(22), keypadHID.button(23)};
    nearLeftTriggers = new Trigger[] {keypadHID.button(13), keypadHID.button(17)};
    nearRightTriggers = new Trigger[] {keypadHID.button(20), keypadHID.button(16)};
    farTriggers = new Trigger[] {keypadHID.button(3), keypadHID.button(2)};
    farLeftTriggers = new Trigger[] {keypadHID.button(5), keypadHID.button(9)};
    farRightTriggers = new Trigger[] {keypadHID.button(12), keypadHID.button(8)};

    // // Move to Pose
    poseButtons(nearTriggers, "Near");
    poseButtons(nearLeftTriggers, "NearLeft");
    poseButtons(nearRightTriggers, "NearRight");
    poseButtons(farTriggers, "Far");
    poseButtons(farLeftTriggers, "FarLeft");
    poseButtons(farRightTriggers, "FarRight");

	//ADD HUMAN PLAYER STATIONS, IN FIELDCONSTANTS :)

	//keypadHID.button(1).whileTrue(grabber.intakeCommand());
	//keypadHID.button(4).whileTrue(grabber.outtakeCommand());
	keypadHID.button(2).onTrue(superstructure.LO()); //switch to human intake

	keypadHID.button(7).onTrue(superstructure.setNextSuperStructure(SuperStructureState.BARGE));
	keypadHID.button(11).onTrue(superstructure.setNextSuperStructure(SuperStructureState.ALG3));
	keypadHID.button(15).onTrue(superstructure.setNextSuperStructure(SuperStructureState.ALG2));
	keypadHID.button(19).onTrue(superstructure.AlgaePickup());
	keypadHID.button(24).onTrue(superstructure.setNextSuperStructure(SuperStructureState.PICKUP));
	// TODO: Add algae pickup to superstructure subsystem (button 24)

	keypadHID.button(18).onTrue(superstructure.setNextSuperStructure(SuperStructureState.LO));
	// keypadHID.button(18).onTrue(setNextSuperStructure(SuperStructureState.L1));
	keypadHID.button(14).onTrue(superstructure.setNextSuperStructure(SuperStructureState.L2));
	keypadHID.button(10).onTrue(superstructure.setNextSuperStructure(SuperStructureState.L3));
	keypadHID.button(6).onTrue(superstructure.setNextSuperStructure(SuperStructureState.L4));

	keypadHID.button(21).onTrue(superstructure.setNextSuperStructure(SuperStructureState.INTAKE));


	keypadHID.button(1).onTrue(grabber.runGrabberCommand(GrabberState.OUTTAKE_C::getSpeed));
	keypadHID.button(4).onTrue(ledSubsystem.setLEDStateCommand(LEDState.ISCLIMBED));


    //MANIPULATOR CONTROLLER

		// Manual Elevator control
		manipulatorController.leftStick()
				.whileTrue(new ManualElevatorCommand(elevator, () -> -manipulatorController.getLeftY() / 50));
		manipulatorController.rightStick()
				.whileTrue(new ManualPivotCommand(pivot, () -> manipulatorController.getRightY()/4));

		//Manual Coral Heights
		manipulatorController.a().onTrue(superstructure.Intake());
		manipulatorController.b().onTrue(superstructure.L2());
		manipulatorController.x().onTrue(superstructure.L3());
		manipulatorController.y().onTrue(superstructure.L4());

		//Manual Algae Heights
		manipulatorController.pov(0).onTrue(superstructure.Barge());
		manipulatorController.pov(270).onTrue(superstructure.Algae3());
		manipulatorController.pov(90).onTrue(superstructure.Algae2());
		manipulatorController.pov(180).onTrue(superstructure.Processor());

		//Intake/Outake
		manipulatorController.leftTrigger().whileTrue(grabber.run(()->grabber.setGrabberState(GrabberState.INTAKE))).onFalse(grabber.run(() -> grabber.setGrabberState(GrabberState.HOLD)));																																																				//fien 
		manipulatorController.rightTrigger().whileTrue(grabber.outtakeCommand());

		// manipulatorController.start().onTrue(grabber.runOnce(() -> pivot.resetPivotAngle(new Rotation2d(3*Math.PI/2))).ignoringDisable(true));		
		manipulatorController.rightBumper().onTrue(superstructure.bargePlace());
		manipulatorController.leftBumper().onTrue(superstructure.LO());
		//reset angles
		// manipulatorController.start().onTrue(new InstantCommand(() -> {
		//   pivot.resetPivotAngle(Rotation2d.kZero);
		//   System.out.println("resetting pivot angle");
		// }));
		// manipulatorController.back().onTrue(new InstantCommand(() -> elevator.resetElevatorEncoders()));

	}

	private void poseButtons(Trigger[] triggers, String name) {
		try {
			triggers[0].whileTrue(alignmentCommandFactory.generateCommand(ReefPoint.valueOf("k" + name + "L"), PathPlannerPath.fromPathFile("WaypointTo" + name + "L")));
			triggers[1].whileTrue(alignmentCommandFactory.generateCommand(ReefPoint.valueOf("k" + name + "R"), PathPlannerPath.fromPathFile("WaypointTo" + name + "R")));
			triggers[0].and(triggers[1]).whileTrue(new ParallelCommandGroup(
					alignmentCommandFactory.generateCommand(ReefPoint.valueOf("k" + name + "C"), PathPlannerPath.fromPathFile("WaypointTo" + name + "C")),
					new SequentialCommandGroup(
						grabber.intakeCommand().until(() -> grabber.hasAlgae()),
						AutoBuilder.followPath(PathPlannerPath.fromPathFile(name + "CPluck"))
					)
				)
			);
		} catch (FileVersionException e) {
			System.out.println("Wrong file version! Message: " + e.getMessage());
		} catch (IOException e) {
			System.out.println("The file cannot be read! Message: " + e.getMessage());
		} catch (ParseException e) {
			System.out.println("The file cannot be parsed! Message: " + e.getMessage());
		}
	}

	public Command getTeleopCommand() {
		return teleopCommand;
	}

	public Command getAutonomousCommand() {
		Command autoCommand = autoManager.getAutoManagerSelected();

		SmartDashboard.putString("Auto Selected", autoManager.getAutoManagerSelected().toString());
		return autoCommand;
	}

	public Command autoPluck() {
		return new SequentialCommandGroup(
				new ParallelCommandGroup(
					//new ManualPivotCommand(pivot, () -> 0.15),
					swerveSubsystem.startEnd(
						() -> swerveSubsystem.swerveDrive.drive(
							new ChassisSpeeds(
								MetersPerSecond.of(-0.4),
								MetersPerSecond.zero(),
								RadiansPerSecond.zero()
							)
						),
						swerveSubsystem::stopModules
					)
				).withTimeout(2),
				superstructure.LO());
	}

}
