package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import lib.frc706.cyberlib.LocalADStarAK;
import lib.frc706.cyberlib.commands.ToPointCommand;
import lib.frc706.cyberlib.subsystems.*;
import frc.robot.auto.AlignToReef;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.constants.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.constants.Constants.GrabberConstants.GrabberState;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
import frc.robot.subsystems.*;

public class AutoCommandManager {

    SendableChooser<Command> autoChooser;

    private static final LoggedNetworkNumber maxVel = new LoggedNetworkNumber("AutoCommandManager/maxVel", 1);
    private static final LoggedNetworkNumber maxAccel = new LoggedNetworkNumber("AutoCommandManager/maxAccel", 1);
    private static final LoggedNetworkNumber maxAngularVel = new LoggedNetworkNumber("AutoCommandManager/maxAngularVel",
            2 * Math.PI);
    private static final LoggedNetworkNumber maxAngularAccel = new LoggedNetworkNumber(
            "AutoCommandManager/maxAngularAccel", 4 * Math.PI);

    public AutoCommandManager(SwerveSubsystem swerveSubsystem, Superstructure superstructure, Grabber grabberSubsystem,
            Climber climberSubsystem) {
        configureNamedCommands(swerveSubsystem, superstructure, grabberSubsystem, climberSubsystem);
        Pathfinding.setPathfinder(new LocalADStarAK());
        // all pathplanner autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("SelectAuto", autoChooser);
        PathfindingCommand.warmupCommand().schedule();
    }

    public SendableChooser<Command> getChooser() {
        return autoChooser;
    }

    public Command getAutoManagerSelected() {
        return autoChooser.getSelected();
    }

    public void configureNamedCommands(SwerveSubsystem swerveSubsystem, Superstructure superstructure,
            Grabber grabberSubsystem, Climber climberSubsystem) { // add more when
                                                                                                       // more
                                                                                                       // subsystems are
                                                                                                       // made

        // NamedCommands.registerCommand("INTAKE",
        //         (grabberSubsystem.runGrabberCommand(() -> GrabberState.INTAKE.getSpeed())
        //         .until(grabberSubsystem::hasAlgae))
        //         .withTimeout(5)
        // );
        NamedCommands.registerCommand("CHECK", Commands.print("TEST FLAG 1"));
        NamedCommands.registerCommand("CHECK2", Commands.print("TEST FLAG 2"));
        NamedCommands.registerCommand("CHECK3", Commands.print("TEST FLAG 3"));

        NamedCommands.registerCommand("INTAKE", grabberSubsystem.intakeCommand().withTimeout(5).finallyDo(() -> grabberSubsystem.setGrabberState(GrabberState.HOLD)));
        // NamedCommands.registerCommand("HOLD", grabberSubsystem.runGrabberCommand(() -> GrabberState.HOLD.getSpeed()).withTimeout(2.0));
        NamedCommands.registerCommand("OUTTAKE_C", grabberSubsystem.startEnd(() -> grabberSubsystem.setGrabberState(GrabberState.OUTTAKE_C), () -> grabberSubsystem.setGrabberState(GrabberState.STOP)).withTimeout(1.0));
        NamedCommands.registerCommand("OUTTAKE_A", grabberSubsystem.startEnd(() -> grabberSubsystem.setGrabberState(GrabberState.OUTTAKE_A), () -> grabberSubsystem.setGrabberState(GrabberState.STOP)).withTimeout(2.0));

        NamedCommands.registerCommand("HUMAN", superstructure.Intake());
        NamedCommands.registerCommand("DEF", superstructure.LO());

        NamedCommands.registerCommand("L1", superstructure.L1().withTimeout(2));
        NamedCommands.registerCommand("L2", superstructure.L2().withTimeout(2));
        NamedCommands.registerCommand("L3", superstructure.L3().withTimeout(2));
        NamedCommands.registerCommand("L4", superstructure.L4().withTimeout(2.5));

        NamedCommands.registerCommand("BARGE", Commands.defer(superstructure::bargePlace, Set.of(superstructure)).withTimeout(3));
        NamedCommands.registerCommand("ALG2", superstructure.Algae2().withTimeout(2));
        NamedCommands.registerCommand("ALG3", superstructure.Algae3().withTimeout(2.25));

        NamedCommands.registerCommand("CLIMBOUT", climberSubsystem.runClimbCommand(() -> -0.8).withTimeout(0.90));

    }

    public static Command pathfindToPoseCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                MetersPerSecond.of(maxVel.get()),
                MetersPerSecondPerSecond.of(maxAccel.get()),
                RadiansPerSecond.of(maxAngularVel.get()),
                RadiansPerSecondPerSecond.of(maxAngularAccel.get()));
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public static Command pathfindToPathCommand(String pathName) {
        PathConstraints constraints = new PathConstraints(
                MetersPerSecond.of(maxVel.get()),
                MetersPerSecondPerSecond.of(maxAccel.get()),
                RadiansPerSecond.of(maxAngularVel.get()),
                RadiansPerSecondPerSecond.of(maxAngularAccel.get()));
        try {
            return AutoBuilder.pathfindThenFollowPath(
                    PathPlannerPath.fromPathFile("WaypointTo" + pathName),
                    constraints);
        } catch (Exception e) {
            e.printStackTrace();
            return new WaitCommand(0);
        }
    }

    public static Command pathfindToReefCommand(String reefPointName) {
        PathConstraints constraints = new PathConstraints(
                MetersPerSecond.of(maxVel.get()),
                MetersPerSecondPerSecond.of(maxAccel.get()),
                RadiansPerSecond.of(maxAngularVel.get()),
                RadiansPerSecondPerSecond.of(maxAngularAccel.get()));

        Command pathfindCommand;

        try {
            pathfindCommand = AutoBuilder.pathfindToPose(
                    new Pose2d(
                            PathPlannerPath.fromPathFile("WaypointTo" + reefPointName).getWaypoints().get(0).anchor(),
                            PathPlannerPath.fromPathFile("WaypointTo" + reefPointName).getIdealStartingState()
                                    .rotation()),
                    constraints);
            System.out.println("it works");
        } catch (Exception e) {
            e.printStackTrace();
            return new WaitCommand(0);
        }

        return pathfindCommand.andThen(
                new ToPointCommand(RobotContainer.swerveSubsystem, ReefPoint.valueOf("k" + reefPointName)::getPose));
    }

    public static Command pathfindThenPIDCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
                MetersPerSecond.of(maxVel.get()),
                MetersPerSecondPerSecond.of(maxAccel.get()),
                RadiansPerSecond.of(maxAngularVel.get()),
                RadiansPerSecondPerSecond.of(maxAngularAccel.get()));

        return AutoBuilder.pathfindToPose(targetPose, constraints)
                .until(() -> RobotContainer.swerveSubsystem.getPose().getTranslation()
                        .getDistance(targetPose.getTranslation()) < 1)
                .andThen(new ToPointCommand(RobotContainer.swerveSubsystem, () -> targetPose));
    }

    private void elevatorCommands(String name, Elevator elevator, Arm pivot) {
        NamedCommands.registerCommand(name, elevator.setLevelCommand(ElevatorLevel.valueOf(name))
                .andThen(new WaitCommand(0.3))
                .andThen(pivot.setPivotPositionCommand(PivotPosition.HI)));
    }
}