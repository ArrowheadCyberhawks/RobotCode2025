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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import lib.frc706.cyberlib.LocalADStarAK;
import lib.frc706.cyberlib.commands.ToPointCommand;
import lib.frc706.cyberlib.subsystems.*;
import frc.robot.auto.AlignToReef;
import frc.robot.commands.SetSuperstructureCommand;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.constants.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.constants.Constants.GrabberConstants.GrabberState;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
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
            Climber climberSubsystem, AlignToReef reef) {
        configureNamedCommands(swerveSubsystem, superstructure, grabberSubsystem, climberSubsystem, reef);
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
            Grabber grabberSubsystem, Climber climberSubsystem, AlignToReef alignmentCommandFactory) { // add more when
                                                                                                       // more
                                                                                                       // subsystems are
                                                                                                       // made
        NamedCommands.registerCommand("INTAKE",
                grabberSubsystem.runGrabberCommand(GrabberState.INTAKE.getSpeed())
                .until(grabberSubsystem::hasAlgae)
                .withTimeout(3.5)
        );
        NamedCommands.registerCommand("OUTTAKE", grabberSubsystem.outtakeCommand().withTimeout(1));

        NamedCommands.registerCommand("HUMAN", superstructure.Intake());
        NamedCommands.registerCommand("DEF", superstructure.LO());

        NamedCommands.registerCommand("L1", superstructure.L1().withTimeout(2));
        NamedCommands.registerCommand("L2", superstructure.L2().withTimeout(2));
        NamedCommands.registerCommand("L3", superstructure.L3().withTimeout(2));
        NamedCommands.registerCommand("L4", superstructure.L4().withTimeout(2.5));

        NamedCommands.registerCommand("BARGE", superstructure.Barge().withTimeout(3));
        NamedCommands.registerCommand("ALG2", superstructure.Algae2().withTimeout(2));

        NamedCommands.registerCommand("CLIMBOUT", climberSubsystem.climbOutCommand());

        NamedCommands.registerCommand("FarLeftL", alignmentCommandFactory.generateCommand(ReefPoint.kFarLeftL).withTimeout(.5));
        NamedCommands.registerCommand("FarLeftR", alignmentCommandFactory.generateCommand(ReefPoint.kFarLeftR).withTimeout(.5));
        NamedCommands.registerCommand("NearLeftL", alignmentCommandFactory.generateCommand(ReefPoint.kNearLeftL).withTimeout(.5));
        NamedCommands.registerCommand("NearLeftR", alignmentCommandFactory.generateCommand(ReefPoint.kNearLeftR).withTimeout(.5));
        NamedCommands.registerCommand("FarR", alignmentCommandFactory.generateCommand(ReefPoint.kFarR).withTimeout(.5));
        NamedCommands.registerCommand("FarC", alignmentCommandFactory.generateCommand(ReefPoint.kFarC).withTimeout(.5));
        NamedCommands.registerCommand("FarL", alignmentCommandFactory.generateCommand(ReefPoint.kFarL).withTimeout(.5));
        NamedCommands.registerCommand("NearR", alignmentCommandFactory.generateCommand(ReefPoint.kNearR).withTimeout(.5));
        NamedCommands.registerCommand("NearC", alignmentCommandFactory.generateCommand(ReefPoint.kNearC).withTimeout(.5));
        NamedCommands.registerCommand("NearL", alignmentCommandFactory.generateCommand(ReefPoint.kNearL).withTimeout(.5));
        NamedCommands.registerCommand("FarRightR", alignmentCommandFactory.generateCommand(ReefPoint.kFarRightR).withTimeout(.5));
        NamedCommands.registerCommand("FarRightC", alignmentCommandFactory.generateCommand(ReefPoint.kFarRightC).withTimeout(.5));
        NamedCommands.registerCommand("FarRightL", alignmentCommandFactory.generateCommand(ReefPoint.kFarRightL).withTimeout(.5));
        NamedCommands.registerCommand("NearRightR", alignmentCommandFactory.generateCommand(ReefPoint.kNearRightR).withTimeout(.5));
        NamedCommands.registerCommand("NearRightC", alignmentCommandFactory.generateCommand(ReefPoint.kNearRightC).withTimeout(.5));
        NamedCommands.registerCommand("NearRightL", alignmentCommandFactory.generateCommand(ReefPoint.kNearRightL).withTimeout(.5));
        NamedCommands.registerCommand("FarLeftR", alignmentCommandFactory.generateCommand(ReefPoint.kFarLeftR).withTimeout(.5));
        NamedCommands.registerCommand("FarLeftC", alignmentCommandFactory.generateCommand(ReefPoint.kFarLeftC).withTimeout(.5));
        NamedCommands.registerCommand("FarLeftL", alignmentCommandFactory.generateCommand(ReefPoint.kFarLeftL).withTimeout(.5));
        NamedCommands.registerCommand("NearLeftR", alignmentCommandFactory.generateCommand(ReefPoint.kNearLeftR).withTimeout(.5));
        NamedCommands.registerCommand("NearLeftC", alignmentCommandFactory.generateCommand(ReefPoint.kNearLeftC).withTimeout(.5));
        NamedCommands.registerCommand("NearLeftL", alignmentCommandFactory.generateCommand(ReefPoint.kNearLeftL).withTimeout(.5));

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