package frc.robot.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
import frc.robot.constants.Constants.PID.PathPlanner;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.commands.SetSuperstructureCommand;
import frc.robot.commons.TagUtils;
import frc.robot.commands.LEDCommand;
import frc.robot.auto.DriveToPose;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Superstructure.*;
import lib.frc706.cyberlib.commands.controller.ControllerRumbleCommand;
import lib.frc706.cyberlib.subsystems.*;

public class AlignToReef {

    private final SwerveSubsystem swerveSubsystem;
    private final boolean superstructureExists;
    public static boolean usePathplanner;
    // Elevator elevator;
    // Pivot pivot;
    Grabber grabber;
    Superstructure superstructure;
    LEDSubsystem leds;

    //private final ReefPoint reef;
    private final double modeVal = 0.5;

    //for more flexibility
    PathConstraints coralPathConstraints = new PathConstraints(1.5, 1, Math.PI, 2*Math.PI);
    PathConstraints algaePathConstraints = new PathConstraints(1.5, 1, Math.PI, 2*Math.PI);
    PathConstraints stationPathConstraints = new PathConstraints(1.5, 1, Math.PI, 2*Math.PI);

    public enum PathOption {
        kFarR,
        kFarC,
        kFarL,
        kNearR,
        kNearC,
        kNearL,
      kFarRightR,
      kFarRightC,
      kFarRightL,
      kNearRightR,
      kNearRightC,
      kNearRightL,
      kFarLeftR,
      kFarLeftC,
      kFarLeftL,
      kNearLeftR,
      kNearLeftC,
      kNearLeftL,
      }

    PathPlannerPath NearL;
    PathPlannerPath NearR;
    PathPlannerPath NearLeftL;
    PathPlannerPath NearLeftR;
    PathPlannerPath NearRightL;
    PathPlannerPath NearRightR;
    PathPlannerPath FarL;
    PathPlannerPath FarR;
    PathPlannerPath FarLeftL;
    PathPlannerPath FarLeftR;
    PathPlannerPath FarRightL;
    PathPlannerPath FarRightR;

    Map<PathOption, Command> scoringPathMap = new HashMap<>(12);

    public AlignToReef(SwerveSubsystem swerveSubsystem, Superstructure superstructure, Grabber grabber) {
        this.swerveSubsystem = swerveSubsystem;
        //this.leds = leds;
        this.grabber = grabber;
        this.superstructure = superstructure;
        superstructureExists = true;
        usePathplanner = true;

            try {
                NearL = PathPlannerPath.fromPathFile("WaypointToNearL");
                NearR = PathPlannerPath.fromPathFile("WaypointToNearR");
                NearLeftL = PathPlannerPath.fromPathFile("WaypointToNearLeftL");
                NearLeftR = PathPlannerPath.fromPathFile("WaypointToNearRightR");
                NearRightL = PathPlannerPath.fromPathFile("WaypointToNearRightL");
                NearRightR = PathPlannerPath.fromPathFile("WaypointToNearRightR");
                FarL = PathPlannerPath.fromPathFile("WaypointToFarL");
                FarR = PathPlannerPath.fromPathFile("WaypointToFarR");
                FarLeftL = PathPlannerPath.fromPathFile("WaypointToFarLeftL");
                FarLeftR = PathPlannerPath.fromPathFile("WaypointToFarleftR");
                FarRightL = PathPlannerPath.fromPathFile("WaypointToFarRightL");
                FarRightR = PathPlannerPath.fromPathFile("WaypointToFarRightR");        
            } catch (Exception e) {
              System.out.println(e.getMessage());
            }


    }
    

    //private final LoggedNetworkNumber modeVal = new LoggedNetworkNumber("AlignToReef/modeVal", 0.75);
    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging")
            .getStructTopic("desired branch", Pose2d.struct).publish();

    /**
     * Brings the robot to a desired Pose, automatically scoring the coral
     * @param reef
     * @return
     */
    public Command generateCommand(ReefPoint reef, PathPlannerPath path) {
        //Crash-proof
        if (reef == null || reef.getPose() == null) {
            return Commands.print("Error: Reef or its pose is null");
        }

        return Commands.defer(() -> {
            desiredBranchPublisher.accept(reef.getPose());
            return autoScoreCommand(reef, path);
            //return getPathFromWaypoint(getWaypointFromBranch(reef));
        }, Set.of());
    }

    public Command generateCommand(ReefPoint reef) {
        //Crash-proof
        if (reef == null || reef.getPose() == null) {
            return Commands.print("Error: Reef or its pose is null");
        }

        return Commands.defer(() -> {
            desiredBranchPublisher.accept(reef.getPose());
            return getPathFromWaypoint(getWaypointFromBranch(reef));
        }, Set.of());
    }
    /***
     * Brings the robot to a desired Pose, automatically scoring the coral
     * @param pose The desired Pose2d
     * @return A command that brings the robot to the desired Pose2d using Pathplanner and Odometry
     */
    public Command generateCommand(Pose2d pose) {       
        //Crash-proof
        if (pose == null || pose == null) {
            return Commands.print("Error: Reef or its pose is null");
        }

        return Commands.defer(() -> {
            desiredBranchPublisher.accept(pose);
            return getPathFromWaypoint(pose);
        }, Set.of());
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        //Turns Pose2ds into waypoints
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(
                    swerveSubsystem.getPose().getTranslation(), getPathVelocityHeading(swerveSubsystem.swerveDrive.getFieldVelocity(), waypoint)),
                    waypoint);
        
        //Crash-proof
        if (waypoints.size() < 2) {
             return Commands.print("ERROR: Not enough waypoints to generate a path ( < 2 waypoints )");
        }
        
        //If the robot is close enough it uses only PID
        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < modeVal) {
            return autoDrive(waypoint);
        }

        // PathConstraints constraints = new PathConstraints(1.75, 1.25,
        //     360, Units.degreesToRadians(180));

        // PathPlannerPath path = new PathPlannerPath(
        //         waypoints,
        //         constraints,
        //         new IdealStartingState(getVelocityMagnitude(swerveSubsystem.swerveDrive.getFieldVelocity()),
        //                 swerveSubsystem.getPose().getRotation()),
        //         new GoalEndState(0.0, waypoint.getRotation()));

        // //we have correct coordinates so we don't flip the path
        // path.preventFlipping = true;

        //Follows the path using pathplanner, brings the superstructure up, and then uses PID to get to the pose
        //return AutoBuilder.followPath(path).andThen(superstructureExists ? autoScore(waypoint) : autoDrive(waypoint));

        //return AutoBuilder.followPath(path).alongWith(getSuperStructure(waypoint)).andThen(autoDrive(waypoint));
        return autoDrive(waypoint).alongWith(getSuperStructure(waypoint));


        //return AutoBuilder.followPath(path).alongWith(Commands.runOnce(() -> superstructure.getNextSuperStructure(Superstructure.nextSuperStructureState)));

        // if (usePathplanner) {
        //     return AutoBuilder.followPath(path).alongWith(getSuperStructure(waypoint)).andThen(autoDrive(waypoint));
        // } else {
        //     return autoDrive(waypoint).alongWith(getSuperStructure(waypoint));
        // }    
    
    }

    private Command autoScoreCommand(ReefPoint reef, PathPlannerPath path) {

        //if close enough it uses PID
        if (swerveSubsystem.getPose().getTranslation().getDistance(path.getStartingHolonomicPose().get().getTranslation()) < 1.00) {
            return autoDrive(reef.getPose()).alongWith(getSuperStructure(reef.getPose()));
        }

        //otherwise brings it to the start of th epath in pathplanner
        return AutoBuilder.pathfindThenFollowPath(path, coralPathConstraints).alongWith(getSuperStructure(reef.getPose()));
    }

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(swerveSubsystem.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();// .rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }

    /**
     * 
     * @return Pathplanner waypoint with direction of travel away from the
     *         associated reef side
     */
    private Pose2d getWaypointFromBranch(ReefPoint reef) {
        return reef.getPose();
    }

    private Command autoDrive(Pose2d waypoint) {
        return Commands.sequence(
            Commands.print("start position PID loop to x: " + waypoint.getX() + "and y: " + waypoint.getY()),
            new DriveToPose(swerveSubsystem, waypoint),
            Commands.print("end position PID loop"));
    }


    public Command switchMode() {
        //too lazy to actuallly do the right thing here lmao
        usePathplanner = !usePathplanner;
        return Commands.none();
    }

    public Command getSuperStructure(Pose2d waypoint) {
        final double distance;

        if (Superstructure.nextSuperStructureState.equals(SuperStructureState.L4)) { //perhaps change to be based off of robot vel
            distance = 2.5;
        } else { //add more later
            distance = 1.5;
        }

        Command structure = 
                new SequentialCommandGroup(
                    new ConditionalCommand(
                        superstructure.getNextSuperStructure(Superstructure.nextSuperStructureState), //brings it up
                        new SequentialCommandGroup( //when not < distance
                            superstructure.getNextSuperStructure(SuperStructureState.LO), //goes to the low position
                            Commands.waitUntil(() -> swerveSubsystem.getPose().getTranslation().getDistance(waypoint.getTranslation()) < distance), //until close enough
                            superstructure.getNextSuperStructure(Superstructure.nextSuperStructureState) //then brings it up
                        ),
                        () -> swerveSubsystem.getPose().getTranslation().getDistance(waypoint.getTranslation()) < distance)

                );
        
        return structure;
    }

}