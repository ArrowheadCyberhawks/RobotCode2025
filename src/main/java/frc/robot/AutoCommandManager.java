package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import lib.frc706.cyberlib.LocalADStarAK;
import lib.frc706.cyberlib.commands.ToPointCommand;
import lib.frc706.cyberlib.subsystems.*;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.GrabberConstants.GrabberPosition;
import frc.robot.Constants.ReefPoint;
import frc.robot.commands.SetSuperstructureCommand;
import frc.robot.subsystems.*;


public class AutoCommandManager {

    SendableChooser<Command> autoChooser;

    private static final LoggedNetworkNumber maxVel = new LoggedNetworkNumber("AutoCommandManager/maxVel", 1);
    private static final LoggedNetworkNumber maxAccel = new LoggedNetworkNumber("AutoCommandManager/maxAccel", 1);
    private static final LoggedNetworkNumber maxAngularVel = new LoggedNetworkNumber("AutoCommandManager/maxAngularVel", 2*Math.PI);
    private static final LoggedNetworkNumber maxAngularAccel = new LoggedNetworkNumber("AutoCommandManager/maxAngularAccel", 4*Math.PI);

    public AutoCommandManager(SwerveSubsystem swerveSubsystem, Elevator elevatorSubsystem, Grabber grabberSubsystem) {
        configureNamedCommands(swerveSubsystem, elevatorSubsystem, grabberSubsystem);
        Pathfinding.setPathfinder(new LocalADStarAK());
        //all pathplanner autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("SelectAuto", autoChooser);
        PathfindingCommand.warmupCommand().schedule();
    }

    public SendableChooser<Command> getChooser() {
        return autoChooser;
    }

    public Command getAutoManagerSelected(){
        return autoChooser.getSelected();
    }

    public void configureNamedCommands(SwerveSubsystem swerveSubsystem, Elevator elevatorSubsystem, Grabber grabberSubsystem) { //add more when more subsystems are made
        NamedCommands.registerCommand("INTAKE", grabberSubsystem.runGrabberCommand(0.75).until(grabberSubsystem::hasCoral).withTimeout(2));
        NamedCommands.registerCommand("OUTTAKE", grabberSubsystem.outtakeCommand().withTimeout(2));

        NamedCommands.registerCommand("HUMAN", new SetSuperstructureCommand(grabberSubsystem, elevatorSubsystem, GrabberPosition.HUMAN::getAngle, ElevatorLevel.HUMAN::getHeight));
        NamedCommands.registerCommand("DEF", new SetSuperstructureCommand(grabberSubsystem, elevatorSubsystem, GrabberPosition.ZERO::getAngle, ElevatorLevel.LO::getHeight));

        NamedCommands.registerCommand("L1", new SetSuperstructureCommand(grabberSubsystem, elevatorSubsystem, GrabberPosition.L1::getAngle, ElevatorLevel.L1::getHeight).withTimeout(2));
        NamedCommands.registerCommand("L2", new SetSuperstructureCommand(grabberSubsystem, elevatorSubsystem, GrabberPosition.PLACE::getAngle, ElevatorLevel.L2::getHeight).withTimeout(2));
        NamedCommands.registerCommand("L3", new SetSuperstructureCommand(grabberSubsystem, elevatorSubsystem, GrabberPosition.PLACE::getAngle, ElevatorLevel.L3::getHeight).withTimeout(2));
        NamedCommands.registerCommand("L4", new SetSuperstructureCommand(grabberSubsystem, elevatorSubsystem, GrabberPosition.L4::getAngle, ElevatorLevel.L4::getHeight).withTimeout(2));
    }

    public static Command pathfindToPoseCommand(Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
            MetersPerSecond.of(maxVel.get()),
            MetersPerSecondPerSecond.of(maxAccel.get()),
            RadiansPerSecond.of(maxAngularVel.get()),
            RadiansPerSecondPerSecond.of(maxAngularAccel.get())
        );
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public static Command pathfindToPathCommand(String pathName) {
        PathConstraints constraints = new PathConstraints(
            MetersPerSecond.of(maxVel.get()),
            MetersPerSecondPerSecond.of(maxAccel.get()),
            RadiansPerSecond.of(maxAngularVel.get()),
            RadiansPerSecondPerSecond.of(maxAngularAccel.get())
        );
        try {
            return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("WaypointTo" + pathName),
                constraints
            );
        } catch (Exception e) {
            e.printStackTrace();
            return new WaitCommand(0);
        }
    }

    public static Command pathfindToReefCommand (String reefPointName) {
        PathConstraints constraints = new PathConstraints(
            MetersPerSecond.of(maxVel.get()),
            MetersPerSecondPerSecond.of(maxAccel.get()),
            RadiansPerSecond.of(maxAngularVel.get()),
            RadiansPerSecondPerSecond.of(maxAngularAccel.get())
        );

        Command pathfindCommand;

        try {
            pathfindCommand = AutoBuilder.pathfindToPose(
            new Pose2d(PathPlannerPath.fromPathFile("WaypointTo" + reefPointName).getWaypoints().get(0).anchor(),
                PathPlannerPath.fromPathFile("WaypointTo" + reefPointName).getIdealStartingState().rotation()),
            constraints);
            System.out.println("it works");
        } catch (Exception e) {
            e.printStackTrace();
            return new WaitCommand(0);
        }

        return pathfindCommand.andThen(
            new ToPointCommand(RobotContainer.swerveSubsystem, ReefPoint.valueOf("k" + reefPointName)::getPose)
        );
    }

    public static Command pathfindThenPIDCommand (Pose2d targetPose) {
        PathConstraints constraints = new PathConstraints(
            MetersPerSecond.of(maxVel.get()),
            MetersPerSecondPerSecond.of(maxAccel.get()),
            RadiansPerSecond.of(maxAngularVel.get()),
            RadiansPerSecondPerSecond.of(maxAngularAccel.get())
        );
    
        return AutoBuilder.pathfindToPose(targetPose, constraints)
            .until(() -> RobotContainer.swerveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 1)
            .andThen(new ToPointCommand(RobotContainer.swerveSubsystem, () -> targetPose)
        );
    }
    
    private void elevatorCommands(String name, Elevator elevator, Grabber grabber) {
     NamedCommands.registerCommand(name, elevator.setLevelCommand(ElevatorLevel.valueOf(name))
        .andThen(new WaitCommand(0.3))
        .andThen(grabber.setPivotPositionCommand(GrabberPosition.HI))
     );
    }
}