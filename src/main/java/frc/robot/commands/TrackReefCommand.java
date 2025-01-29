package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.SwerveConstants;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TrackReefCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final PIDController m_turningController = new PIDController(PID.kPAutoTurning, PID.kIAutoTurning, PID.kDAutoTurning);
    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final double maxAngularVel;
    private Pose2d target;

    public TrackReefCommand(SwerveSubsystem swerveSubsystem, Pose2d target,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.target = target;
        maxAngularVel = SwerveConstants.maxTrackingAngularVel;
        field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        addRequirements(swerveSubsystem);
    }

    public static Pose2d getTagPose(int tagId) {
        return field.getTagPose(tagId).orElse(new Pose3d()).toPose2d();
    }

    public void setTargetPose(Pose2d newTarget) {
        target = newTarget;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs
        double xSpeed = ySpdFunction.get();
        double ySpeed = xSpdFunction.get();

        //Apply deadband
        xSpeed = MathUtil.applyDeadband(xSpeed, IOConstants.kDriverControllerDeadband);
        ySpeed = MathUtil.applyDeadband(ySpeed, IOConstants.kDriverControllerDeadband);
        //Make driving smoother
        xSpeed *= Math.abs(xSpeed);
        ySpeed *= Math.abs(ySpeed);

        // MONKEY CODE (made by our fellow monkey)
        //targetAngle = Math.atan2(targetPose.getTranslation().getY()-robotPose.getTranslation().getY(), targetPose.getTranslation().getX()-robotPose.getTranslation().getX());
        
        double turningSpeed = m_turningController.calculate(calculateAngleTo(target, swerveSubsystem.getPose()));
        turningSpeed = MathUtil.clamp(turningSpeed, -maxAngularVel, maxAngularVel);

        // x *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		// y *= MathUtil.interpolate(0.15, 1, accelMultiplier);

        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
    }

    /**
     * Calculate the difference between the robot's current angle and the angle required to point at a specified location.
     */
    public static double calculateAngleTo(Pose2d targetPose, Pose2d robotPose) {
        double targetAngle = targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        return MathUtil.angleModulus(robotPose.getRotation().getRadians()-targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}