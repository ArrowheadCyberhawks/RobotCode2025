package frc.robot.commons;

import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.constants.Constants.ReefPoint;

public class TagUtils {
    private static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final Transform2d leftReefTransform = new Transform2d(Inches.of(18), Inches.of(-12), new Rotation2d(Math.PI)); //-6.5
    private static final Transform2d centerReefTransform = new Transform2d(Inches.of(18), Inches.of(-5), new Rotation2d(Math.PI));
    private static final Transform2d rightReefTransform = new Transform2d(Inches.of(18), Inches.of(1.5), new Rotation2d(Math.PI));//6.5
    

    public static Pose2d getTagPose(int tagId) {
        field.setOrigin(OriginPosition.kBlueAllianceWallRightSide); //this shouldn't be necessary
        return field.getTagPose(tagId).orElse(new Pose3d()).toPose2d();
    }

    public static Pose2d getOffsetRightAprilTag(int tagId) {
        return getTagPose(tagId).transformBy(rightReefTransform);
    }

    public static Pose2d getOffsetCenterAprilTag(int tagId) {
        return getTagPose(tagId).transformBy(centerReefTransform);
    }

    public static Pose2d getOffsetLeftAprilTag(int tagId) {
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
}
