package frc.robot.auto;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.Constants.ReefPoint;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperStructureState;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;
import frc.robot.commons.LoggedTunableNumber;

import java.util.ArrayList;
import java.util.List;

public class AutoCycle extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Superstructure superstructure;
  private final Grabber grabber;

  private final List<Pose2d> reefPoints;
  public int[][] algaeArray = new int[12][4];
  private final AlignToReef alignmentCommandFactory;

  private int scoredL4 = 0;
  private int scoredL3 = 0;
  private int scoredL2 = 0;
  private int scoredL1 = 0;

  public AutoCycle(SwerveSubsystem swerveSubsystem, Superstructure superstructure, Grabber grabber) {
    this.swerveSubsystem = swerveSubsystem;
    this.superstructure = superstructure;
    this.grabber = grabber;
    reefPoints = convertReefPointsToPoses(getReefPoints());
    initScoringArray();

    alignmentCommandFactory = new AlignToReef(this.swerveSubsystem, this.superstructure, this.grabber);

    addRequirements(swerveSubsystem, superstructure, grabber);
    setName("AutoCycle");
  }

  public Command run() {
    Pose2d targetPose = getNextBestScorePose(false);
    // clearAlgaeIfNecessary(targetPose);
    alignmentCommandFactory.generateCommand(targetPose);
    return returnToHumanPlayerStation();
  }

  private Pose2d getNextBestScorePose(boolean coopertitionActive) {
    Pose2d nextPose = null;

    int L4_THRESHOLD = 5;
    int L3_THRESHOLD = 5;
    int L2_THRESHOLD = 5;
    int L1_THRESHOLD = 5;
    //change to not use so many ifs
    if (scoredL4 < L4_THRESHOLD) {
      nextPose = findClosestPoseWithStatus("L4", 0);
    } else if (scoredL3 < L3_THRESHOLD) {
      nextPose = findClosestPoseWithStatus("L3", 0);
    } else if (scoredL2 < L2_THRESHOLD) {
      nextPose = findClosestPoseWithStatus("L2", 0);
    } else if (scoredL1 < L1_THRESHOLD && !coopertitionActive) {
      nextPose = findClosestPoseWithStatus("L1", 0);
    } else {
      nextPose = findClosestPoseWithStatus("L4", 0);
    }
    return nextPose;
  }

  private Pose2d findClosestPoseWithStatus(String level, int status) {
    Pose2d closestPose = null;
    double closestDistance = Double.MAX_VALUE;

    if (level.equals("L4")) {
      Superstructure.nextSuperStructureState = SuperStructureState.L4;
    } else if (level.equals("L3")) {
      Superstructure.nextSuperStructureState = SuperStructureState.L3;
    } else if (level.equals("L2")) {
      Superstructure.nextSuperStructureState = SuperStructureState.L2;
    } else if (level.equals("L1")) {
      Superstructure.nextSuperStructureState = SuperStructureState.L1;
    }

    // USE ONCE IF STATEMENT BONZONA WORKS
    // Superstructure.nextSuperStructureState = SuperStructureState.valueOf(level);

    for (int i = 0; i < reefPoints.size(); i++) {
      Pose2d nextPose = reefPoints.get(i);
      int currentStatus = algaeArray[i][levelToIndex(level)];

      if (currentStatus == status) {
        double distance = nextPose.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation());

        if (distance < closestDistance) {
          closestPose = nextPose;
          closestDistance = distance;
        }
      }
    }
    return closestPose;
  }

  // private void clearAlgaeIfNecessary(Pose2d targetPose) {
  // int x = reefPoints.indexOf(targetPose);
  // if (algaeArray[x][1] == 2 || algaeArray[x][2] == 2) {
  // algaeArray[x][1] = 0;
  // algaeArray[x][2] = 0;
  // AlignmentCommand clearAlgaeCommand =
  // alignmentCommandFactory.generateCommand(targetPose);
  // swerveSubsystem.setAlignmentCommand(clearAlgaeCommand);
  // }
  // }

  private Command returnToHumanPlayerStation() {
    Pose2d humanPlayerStationPose = findClosestHumanPlayerStation();
    return alignmentCommandFactory.generateCommand(humanPlayerStationPose);
  }

  private Pose2d findClosestHumanPlayerStation() {

    Pose2d leftHuman = new Pose2d(
      16.697198,
      0.65532,
      new Rotation2d(Units.degreesToRadians(30)));

    Pose2d rightHuman = new Pose2d(
        16.697198,
        7.396,
        new Rotation2d(Units.degreesToRadians(30)));
        
    if (leftHuman.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation()) < rightHuman.getTranslation().getDistance(swerveSubsystem.getPose().getTranslation())) {
      return leftHuman;
    } else {
      return rightHuman;
    }
  }

  private void logScoring() {
    Logger.recordOutput("Scoring/L4", scoredL4);
    Logger.recordOutput("Scoring/L3", scoredL3);
    Logger.recordOutput("Scoring/L2", scoredL2);
    Logger.recordOutput("Scoring/L1", scoredL1);
  }

  public List<Pose2d> convertReefPointsToPoses(List<ReefPoint> reefPoints) {
    List<Pose2d> poseList = new ArrayList<>();
    for (ReefPoint point : reefPoints) {
      poseList.add(point.getPose());
    }
    return poseList;
  }

  public List<ReefPoint> getReefPoints() {
    List<ReefPoint> reefPoints = new ArrayList<>();
    for (ReefPoint point : ReefPoint.values()) {
      reefPoints.add(point);
    }
    reefPoints.remove(12);
    return reefPoints;
  }

  public void initScoringArray() {
    algaeArray = new int[][] {
        { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 },
        { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 },
        { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }, { 0, 2, 2, 0 }
    };
  }

  private int levelToIndex(String level) {
    switch (level) {
      case "L4":
        return 0;
      case "L3":
        return 1;
      case "L2":
        return 2;
      case "L1":
        return 3;
      default:
        return -1;
    }
  }

}
