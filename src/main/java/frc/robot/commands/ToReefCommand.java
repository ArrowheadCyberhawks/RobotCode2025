package frc.robot.commands;

import static frc.robot.Constants.PID;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToReefCommand extends TrackPointCommand {
    
    private static PIDController xController, yController;
    private Supplier<Pose2d> targetSupplier;

    public ToReefCommand(SwerveSubsystem swerveSubsystem, Pose2d target) {
        super(swerveSubsystem, target,
            () -> {return -calculateXSpeed(swerveSubsystem.getPose(), target);},
            () -> {return -calculateYSpeed(swerveSubsystem.getPose(), target);}//.transformBy(new Transform2d(0, swerveSubsystem.swerveDrive.swerveDriveConfiguration.getTracklength()/2,new Rotation2d()))
        );
        xController = new PIDController(PID.PointTrack.kPX, PID.PointTrack.kIX, PID.PointTrack.kDX);
        yController = new PIDController(PID.PointTrack.kPY, PID.PointTrack.kIY, PID.PointTrack.kDY);
    //monkey ah code
    }

    public ToReefCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetSupplier) {
        this(swerveSubsystem, targetSupplier.get());
        this.targetSupplier = targetSupplier;
    }

    private static double calculateXSpeed(Pose2d currentPose, Pose2d targetPose) {
        return xController.calculate(calculatePolarAngleTo(currentPose, targetPose), 0)/(Math.abs(TrackPointCommand.calculateAngleTo(currentPose, targetPose))+1);
    }

    private static double calculateYSpeed(Pose2d currentPose, Pose2d targetPose) {
        return yController.calculate(currentPose.getTranslation().getDistance(targetPose.getTranslation()), 0.25)/(Math.abs(TrackPointCommand.calculateAngleTo(currentPose, targetPose))+1);
    }
    /**
     * Calculates the angle to point the robot towards the target
     * @param currentPose current pose of the robot
     * @param targetPose pose of the apriltag (or whatever else we want to point towards)
     * @return angle between the robot and the vector facing into the front of the target
     */
    private static double calculatePolarAngleTo(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getRotation().minus(targetPose.getRotation()).minus(Rotation2d.kPi).getRadians();
    }

    @Override
    public void initialize() {
        super.initialize();
        if (targetSupplier != null) {
            target = targetSupplier.get();
            super.setTargetPose(target);
        }
    }

    @Override
    public void execute() {
        super.execute();
        System.out.println(Units.radiansToDegrees(TrackPointCommand.calculateAngleTo(super.swerveSubsystem.getPose(), target)));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        //robot needs to kill itself at some point
        return super.swerveSubsystem.getPose().getTranslation().getDistance(target.getTranslation()) < 0.27;
    }
}