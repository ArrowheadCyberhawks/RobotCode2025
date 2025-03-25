package frc.robot.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import frc.robot.constants.Constants;
import lib.frc706.cyberlib.subsystems.*;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.commons.GeomUtil;
import frc.robot.commons.LoggedTunableNumber;

public class DriveToPose extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Pose2d> targetPose;

  //init pid controllers
  //TODO Tune DriveToPose PID
  private final ProfiledPIDController driveController =  new ProfiledPIDController(Constants.PID.Auto.kPTranslation, Constants.PID.Auto.kITranslation, Constants.PID.Auto.kDTranslation, new Constraints(3, 1));
  private final PIDController headingController = new PIDController(Constants.PID.Auto.kThetaPIDConstants.kP, Constants.PID.Auto.kThetaPIDConstants.kI, Constants.PID.Auto.kThetaPIDConstants.kD);

  private double driveErrorAbs;
  private Translation2d lastSetpointTranslation;

  private Optional<DoubleSupplier> yOverride = Optional.empty();
  private Optional<Supplier<Translation2d>> translationOverride = Optional.empty();

  //Different constructors
  public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPose) {
    this.swerveSubsystem = swerveSubsystem;
    this.targetPose = targetPose;
    addRequirements(swerveSubsystem);
    setName("DriveToPose");
  }

  public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPose, DoubleSupplier yOverride) {
    this(swerveSubsystem, targetPose);
    this.yOverride = Optional.of(yOverride);
  }

  public DriveToPose(SwerveSubsystem swerveSubsystem, Supplier<Pose2d> targetPose,
      Supplier<Translation2d> translationOverride) {
    this(swerveSubsystem, targetPose);
    this.translationOverride = Optional.of(translationOverride);
  }

  public DriveToPose(SwerveSubsystem swerveSubsystem, Pose2d targetPose) {
    this(swerveSubsystem, () -> targetPose);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveSubsystem.getPose();

    //reset drive controller with current vel and distance
    //by getting the current velocity it makes it smoother
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(swerveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond,
                swerveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond)
                .rotateBy(
                    targetPose.get().getTranslation().
                    minus(swerveSubsystem.getPose().getTranslation()).getAngle().
                    unaryMinus()
                  ).getX()));
    headingController.reset();

    //sets initial setpoint
    lastSetpointTranslation = currentPose.getTranslation();
  }

  @Override
  public void execute() {
    //sets current and target pose values
    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d targetPose = this.targetPose.get();

    //gets translation between target and current pose (distance formula)
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    //scales speed based on distance from target
    double ffScaler = MathUtil.clamp((currentDistance - 0.2) / (0.8 - 0.2), 0.0, 1.0);
    driveErrorAbs = currentDistance;


    lastSetpointTranslation = new Pose2d(
        targetPose.getTranslation(),
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
        .transformBy(
            GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
        .getTranslation();

    
    double driveVelocity = driveController.getSetpoint().velocity * ffScaler + driveController.calculate(driveErrorAbs, 0.0);
    double headingVelocity = headingController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    Translation2d velocity;

    //takes into account any overrides
    if (translationOverride.isPresent() && translationOverride.get().get().getNorm() > 0.5) {
      velocity = translationOverride.get().get();
    } else {
      //sets velocity to the movement direction
      velocity = new Pose2d(
          new Translation2d(),
          currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
          .transformBy(GeomUtil.translationToTransform(driveVelocity, 0.0))
          .getTranslation();
    }

    //gives next movement to swerve drive
    swerveSubsystem.driveFieldOriented(new ChassisSpeeds(velocity.getX(), yOverride.isPresent() ? yOverride.get().getAsDouble() : velocity.getY(), headingVelocity));

    Logger.recordOutput("DriveToPose/MeasuredDistance", currentDistance);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPose/MeasuredHeading", currentPose.getRotation().getDegrees());
    Logger.recordOutput("DriveToPose/SetpointHeading", targetPose.getRotation().getDegrees());
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d(lastSetpointTranslation, new Rotation2d(headingController.getSetpoint())));
    Logger.recordOutput("DriveToPose/TargetPose", targetPose);
  }

  public boolean atGoal() {
    return driveController.atGoal() && headingController.atSetpoint();
  }
}