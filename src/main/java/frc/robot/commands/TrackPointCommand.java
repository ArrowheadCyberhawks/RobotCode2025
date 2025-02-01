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

/**
 * Command to point towards a location on the field using the swerve drive.
 */
public class TrackPointCommand extends Command {

    protected final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSupplier, ySupplier, accelSupplier;
    private final PIDController m_turningController = new PIDController(PID.PointTrack.kPAutoTurning, PID.PointTrack.kIAutoTurning, PID.PointTrack.kDAutoTurning);
    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final double maxAngularVel;
    private static final Transform2d leftReefTransform = new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(-6.5), new Rotation2d(0));
    private static final Transform2d rightReefTransform = new Transform2d(Units.inchesToMeters(0), Units.inchesToMeters(6.5), new Rotation2d(0));
    protected Pose2d target;
    protected Supplier<Pose2d> targetSupplier;
    private final boolean controllerCorrections;
    private final double kMaxVel;

    /**
     * Enum to represent different common field positions.
     */
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
    //monkey ah code
    /**
     * Enum to represent branches of the reef.
     */
    public static enum ReefPoint {
        kCenter(new Pose2d(13, 4, new Rotation2d()), new Pose2d(4.5, 4, new Rotation2d())),
        kFarR(getOffsetRightAprilTag(10), getOffsetRightAprilTag(21)),
        kFarC(getTagPose(10), getTagPose(21)),
        kFarL(getOffsetLeftAprilTag(10), getOffsetLeftAprilTag(21)),
        kNearR(getOffsetRightAprilTag(7), getOffsetRightAprilTag(18)),
        kNearC(getTagPose(7), getTagPose(18)),
        kNearL(getOffsetLeftAprilTag(7), getOffsetLeftAprilTag(18)),
        kFarRightR(getOffsetRightAprilTag(9), getOffsetRightAprilTag(22)),
        kFarRightC(getTagPose(9), getTagPose(22)),
        kFarRightL(getOffsetLeftAprilTag(9), getOffsetLeftAprilTag(22)),
        kNearRightR(getOffsetRightAprilTag(8), getOffsetRightAprilTag(17)),
        kNearRightC(getTagPose(8), getTagPose(17)),
        kNearRightL(getOffsetLeftAprilTag(8), getOffsetLeftAprilTag(17)),
        kFarLeftR(getOffsetRightAprilTag(11), getOffsetRightAprilTag(20)),
        kFarLeftC(getTagPose(11), getTagPose(20)),
        kFarLeftL(getOffsetLeftAprilTag(11), getOffsetLeftAprilTag(20)),
        kNearLeftR(getOffsetRightAprilTag(6), getOffsetRightAprilTag(19)),
        kNearLeftC(getTagPose(6), getTagPose(19)),
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
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction, double maxVel, boolean controllerCorrections) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSpdFunction;
        this.ySupplier = ySpdFunction;
        this.accelSupplier = accelFunction;
        this.target = target;
        this.controllerCorrections = controllerCorrections;
        kMaxVel = maxVel;
        maxAngularVel = SwerveConstants.maxTrackingAngularVel;
        field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        addRequirements(swerveSubsystem);
    }

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, Pose2d target,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction) {
        this(swerveSubsystem, target, xSpdFunction, ySpdFunction, accelFunction, SwerveConstants.kMaxVelTele, true);
    }

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, Pose2d target,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this(swerveSubsystem, target, xSpdFunction, ySpdFunction, ()-> 0.0, SwerveConstants.kMaxAccelAuto, false);
    }

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetSupplier,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction) {
        this(swerveSubsystem, targetSupplier.get(), xSpdFunction, ySpdFunction, accelFunction, SwerveConstants.kMaxVelTele, true);
        this.targetSupplier = targetSupplier;
    }

    public TrackPointCommand(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetSupplier,
        Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction) {
        this(swerveSubsystem, targetSupplier.get(), xSpdFunction, ySpdFunction, ()-> 0.0, SwerveConstants.kMaxAccelAuto, false);
        this.targetSupplier = targetSupplier;
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
    public static ReefPoint getClosestReefPoint(Pose2d pose) {
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
        System.out.println("Closest reef point: " + closestReefPoint + closestReefPoint.getPose());
        return closestReefPoint;
    }

    @Override
    public void execute() {
        //Get real-time joystick inputs
        double xInput = ySupplier.get();
        double yInput = xSupplier.get();
        double accelMultiplier = accelSupplier.get();
        double xSpeed = 0, ySpeed = 0;
        if(controllerCorrections) {
            //Apply deadband
            xInput = MathUtil.applyDeadband(xInput, IOConstants.kDriverControllerDeadband);
            yInput = MathUtil.applyDeadband(yInput, IOConstants.kDriverControllerDeadband);
            
            //Make driving smoother
            xInput *= Math.abs(xInput);
            yInput *= Math.abs(yInput);

            xInput *= MathUtil.interpolate(0.15, 1, accelMultiplier);
            yInput *= MathUtil.interpolate(0.15, 1, accelMultiplier);

            xSpeed = xInput * kMaxVel;
            ySpeed = yInput * kMaxVel;
            
        } else {
            xSpeed = MathUtil.clamp(xInput, -kMaxVel, kMaxVel);
            ySpeed = MathUtil.clamp(yInput, -kMaxVel, kMaxVel);
        }

        // MONKEY CODE (made by our fellow monkey)
        //targetAngle = Math.atan2(targetPose.getTranslation().getY()-robotPose.getTranslation().getY(), targetPose.getTranslation().getX()-robotPose.getTranslation().getX());
        
        double turningSpeed = m_turningController.calculate(calculateAngleTo(swerveSubsystem.getPose(), target));
        turningSpeed = MathUtil.clamp(turningSpeed, -maxAngularVel, maxAngularVel);

        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(new ChassisSpeeds(xSpeed, ySpeed, turningSpeed));
    }

    /**
     * Calculate the difference between the robot's current angle and the angle required to point at a specified location.
     * 
     * @param robotPose The current pose of the robot.
     * @param targetPose The pose of the target location.
     * @return The angle difference in radians.
     */
    public static double calculateAngleTo(Pose2d robotPose, Pose2d targetPose) {
        double targetAngle = targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getRadians();
        return MathUtil.angleModulus(robotPose.getRotation().getRadians()-targetAngle);
    }

    @Override
    public void initialize() {
        if (targetSupplier != null) {
            target = targetSupplier.get();
        }
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