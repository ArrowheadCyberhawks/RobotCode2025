package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PID;
import frc.robot.Constants.SwerveConstants;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TrackPointCommand extends Command {

    protected final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSupplier, ySupplier, accelSupplier;
    private final PIDController m_turningController = new PIDController(PID.PointTrack.kPAutoTurning, PID.PointTrack.kIAutoTurning, PID.PointTrack.kDAutoTurning);
    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final double maxAngularVel;
    private static final Transform2d leftReefTransform = new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(-6.5), new Rotation2d(0));
    private static final Transform2d rightReefTransform = new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(6.5), new Rotation2d(0));
    protected Pose2d target;
    private final double kMaxVel;

    public static enum FieldPosition { //nitin don't touch this either I DON'T WANT IT PRETTIER
        kBargeLeft(new Pose2d(8.775, 0.75, new Rotation2d()), new Pose2d(8.775, 7.25, new Rotation2d())),
        kBargeMiddle(new Pose2d(8.775, 1.9, new Rotation2d()), new Pose2d(8.775, 6.16, new Rotation2d())),
        kBargeRight(new Pose2d(8.775, 3, new Rotation2d()), new Pose2d(8.775, 5, new Rotation2d())),
        kLeftCoralStation(new Pose2d(16.75, 0.65, new Rotation2d()), new Pose2d(0.75, 7.35, new Rotation2d())),
        kRightCoralStation(new Pose2d(16.75, 7.35, new Rotation2d()), new Pose2d(0.75, 0.65, new Rotation2d()));

        public final Pose2d redPose, bluePose;

        private FieldPosition(Pose2d redPose, Pose2d bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        public Pose2d getPose() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
        }
    }

    public static enum ReefPoint {
        kCenter(new Pose2d(13, 4, new Rotation2d()), new Pose2d(4.5, 4, new Rotation2d())),
        kFarR(getOffsetRightAprilTag(10), getOffsetRightAprilTag(21)),
        kFarL(getOffsetLeftAprilTag(10), getOffsetLeftAprilTag(21)),
        kNearR(getOffsetRightAprilTag(7), getOffsetRightAprilTag(18)),
        kNearL(getOffsetLeftAprilTag(7), getOffsetLeftAprilTag(18)),
        kFarRightR(getOffsetRightAprilTag(9), getOffsetRightAprilTag(22)),
        kFarRightL(getOffsetLeftAprilTag(9), getOffsetLeftAprilTag(22)),
        kNearRightR(getOffsetRightAprilTag(8), getOffsetRightAprilTag(17)),
        kNearRightL(getOffsetLeftAprilTag(8), getOffsetLeftAprilTag(17)),
        kFarLeftR(getOffsetRightAprilTag(11), getOffsetRightAprilTag(20)),
        kFarLeftL(getOffsetLeftAprilTag(11), getOffsetLeftAprilTag(20)),
        kNearLeftR(getOffsetRightAprilTag(6), getOffsetRightAprilTag(19)),
        kNearLeftL(getOffsetLeftAprilTag(6), getOffsetLeftAprilTag(19));

        public final Pose2d redPose, bluePose;

        private ReefPoint(Pose2d redPose, Pose2d bluePose) {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }

        public Pose2d getPose() {
            return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? redPose : bluePose;
        }
    }

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, Pose2d target,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction, double maxVel) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSpdFunction;
        this.ySupplier = ySpdFunction;
        this.accelSupplier = accelFunction;
        this.target = target;
        kMaxVel = maxVel;
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

    private static Pose2d getOffsetRightAprilTag(int tagId) {
        return getTagPose(tagId).transformBy(rightReefTransform);
    }

    private static Pose2d getOffsetLeftAprilTag(int tagId) {
        return getTagPose(tagId).transformBy(leftReefTransform);
        }

    /**
     * Finds the closest ReefPoint to the given Pose2d.
     *
     * @param pose The current pose to compare against the reef points.
     * @return The closest ReefPoint to the given pose. If no reef points are found, returns ReefPoint.kCenter.
     */
    public ReefPoint getClosestReefPoint(Pose2d pose) {
        ReefPoint closestReefPoint = ReefPoint.kCenter;
        double minDistance = pose.getTranslation().getDistance(ReefPoint.kCenter.getPose().getTranslation());

        for (ReefPoint reefPoint : ReefPoint.values()) {
            Pose2d reefPose = reefPoint.getPose();
            double distance = pose.getTranslation().getDistance(reefPose.getTranslation());

            if (distance < minDistance) {
            minDistance = distance;
            closestReefPoint = reefPoint;
            }
        }

        return closestReefPoint;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs
        double xInput = ySupplier.get();
        double yInput = xSupplier.get();
        double accelMultiplier = accelSupplier.get();

        //Apply deadband
        xInput = MathUtil.applyDeadband(xInput, IOConstants.kDriverControllerDeadband);
        yInput = MathUtil.applyDeadband(yInput, IOConstants.kDriverControllerDeadband);
        
        //Make driving smoother
        xInput *= Math.abs(xInput);
        yInput *= Math.abs(yInput);

        // MONKEY CODE (made by our fellow monkey)
        //targetAngle = Math.atan2(targetPose.getTranslation().getY()-robotPose.getTranslation().getY(), targetPose.getTranslation().getX()-robotPose.getTranslation().getX());
        
        double turningSpeed = m_turningController.calculate(calculateAngleTo(target, swerveSubsystem.getPose()));
        turningSpeed = MathUtil.clamp(turningSpeed, -maxAngularVel, maxAngularVel);

        xInput *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		yInput *= MathUtil.interpolate(0.15, 1, accelMultiplier);

        double xSpeed = xInput * kMaxVel;
        double ySpeed = yInput * kMaxVel;

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