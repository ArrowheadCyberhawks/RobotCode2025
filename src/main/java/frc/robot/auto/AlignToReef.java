package frc.robot.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.constants.FieldConstants;
import frc.robot.auto.DriveToPose;
import frc.robot.subsystems.*;
import lib.frc706.cyberlib.subsystems.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import org.littletonrobotics.junction.Logger;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.LoggedTunableNumber;

public class AlignToReef {

    private final SwerveSubsystem swerveSubsystem;
    //private final ReefPoint reef;
    // private final double modeVal = 0.3;

    public AlignToReef(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        //this.reef = reef;
    }

    private final LoggedNetworkNumber modeVal = new LoggedNetworkNumber("AlignToReef/modeVal", 0.5);
    private final StructPublisher<Pose2d> desiredBranchPublisher = NetworkTableInstance.getDefault().getTable("logging")
            .getStructTopic("desired branch", Pose2d.struct).publish();

    public Command generateCommand(ReefPoint reef) {
        return Commands.defer(() -> {
            desiredBranchPublisher.accept(reef.getPose());
            return getPathFromWaypoint(getWaypointFromBranch(reef));
        }, Set.of());
    }

    private Command getPathFromWaypoint(Pose2d waypoint) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(swerveSubsystem.getPose().getTranslation(),
                        getPathVelocityHeading(swerveSubsystem.swerveDrive.getFieldVelocity(), waypoint)),
                waypoint);

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < modeVal.get()) {
            return Commands.sequence(
                    Commands.print("start position PID loop"),
                    new DriveToPose(swerveSubsystem, waypoint),
                    Commands.print("end position PID loop"));
        }

        PathConstraints constraints = new PathConstraints(
                swerveSubsystem.swerveDrive.getMaximumChassisVelocity(), 4.0,
                swerveSubsystem.swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(getVelocityMagnitude(swerveSubsystem.swerveDrive.getFieldVelocity()),
                        swerveSubsystem.getPose().getRotation()),
                new GoalEndState(0.0, waypoint.getRotation()));

        path.preventFlipping = true;

        return AutoBuilder.followPath(path).andThen(
                Commands.print("Starting position PID loop"),
                new DriveToPose(swerveSubsystem, waypoint),
                Commands.print("Ending position PID loop"));
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

}