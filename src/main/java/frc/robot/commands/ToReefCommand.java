package frc.robot.commands;

import static frc.robot.Constants.PID;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToReefCommand extends TrackPointCommand {
    private static PIDController xController, yController;

    public ToReefCommand(SwerveSubsystem swerveSubsystem, Pose2d target, double maxVel) {
        super(swerveSubsystem, target,
            () -> {return -calculateXSpeed(swerveSubsystem.getPose(), target);},
            () -> {return -calculateYSpeed(swerveSubsystem.getPose(), target.transformBy(new Transform2d(swerveSubsystem.swerveDrive.swerveDriveConfiguration.getTracklength()/2, 0, new Rotation2d())));},
            () -> 0.05,
            maxVel
        );
        xController = new PIDController(PID.PointTrack.kPX, PID.PointTrack.kIX, PID.PointTrack.kDX);
        yController = new PIDController(PID.PointTrack.kPY, PID.PointTrack.kIY, PID.PointTrack.kDY);

    }

    private static double calculateXSpeed(Pose2d currentPose, Pose2d targetPose) {
        return xController.calculate(calculateAngleToTarget(currentPose, targetPose), 0);
    }

    private static double calculateYSpeed(Pose2d currentPose, Pose2d targetPose) {
        return yController.calculate(currentPose.getTranslation().getDistance(targetPose.getTranslation()), 0);
    }

    /**
     * Calculates the angle to point the robot towards the target
     * @param currentPose current pose of the robot
     * @param targetPose pose of the apriltag (or whatever else we want to point towards)
     * @return angle between the robot and the vector facing into the front of the target
     */
    private static double calculateAngleToTarget(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians() - Math.PI;
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        //robot needs to kill itself at some point
        return super.swerveSubsystem.getPose().getTranslation().getDistance(target.getTranslation()) < 0.05;
    }
}