// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */



public final class Constants {
  public static class IOConstants {
    public static final int kDriverControllerPortUSB = 0;
    public static final int kManipulatorControllerPortUSB = 1;
    public static final int kDriverControllerPortBT = 2;
    public static final int kManipulatorControllerPortBT = 3;

    public static final int kKeypadPort = 5;

    public static final double kManipulatorJoystickDeadband = 0.05;
    public static final double kDriverControllerDeadband = 0.07;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorPort = 13;
    public static final int kExtendMotorPort = 12;
  }

  public static class ClimberConstants {
    public static final int kClimberMotorPort = 1; // change later 

    public static final LoggedNetworkNumber kClimbP = new LoggedNetworkNumber("Climb/kPivotP", 0.1); //change later 
    public static final LoggedNetworkNumber kClimbI = new LoggedNetworkNumber("Climb/kPivotI", 0);
    
    public static final LoggedNetworkNumber kClimbMaxVel = new LoggedNetworkNumber("Climb/kElevatorMaxVel", 100);
    public static final LoggedNetworkNumber kClimbMaxAccel = new LoggedNetworkNumber("Climb/kElevatorMaxAccel", 100);

  }

  public static class ElevatorConstants {
    public static final LoggedNetworkNumber kElevatorP = new LoggedNetworkNumber("Elevator/kElevatorP", 0.08);
    public static final LoggedNetworkNumber kElevatorMaxVel = new LoggedNetworkNumber("Elevator/kElevatorMaxVel", 2000);
    public static final LoggedNetworkNumber kElevatorMaxAccel = new LoggedNetworkNumber("Elevator/kElevatorMaxAccel", 5000);

    public static final int elevatorMotorID = 9;

    public static enum ElevatorLevel { //TODO update positions
      LO(0.0),  //x is about 1.143 cm //0
      L1(10.0), //40 // 22
      L2(40.0), //60 //50 / 22 
      L3(80.0), //80
      L4(120.0), //100
      HI(148.0),
      DEF(60), // 155
      PICK(40);

      private final double height;

      private ElevatorLevel(double height) {
          this.height = height;
      }

      public double getHeight() {
          return height;
      }
    }
  }

  public static class GrabberConstants {
    public static final int kGrabberMotor1Port = 10;
    public static final int kGrabberMotor2Port = 12;
    public static final int kPivotMotorPort = 11;

    public static final int kCoralSensorPort = 13;
    public static final int kAlgaeSensorPort = 22;

    public static final double kCoralSensorThreshold = 100;
    public static final double kAlgaeSensorThreshold = 100;

    public static final LoggedNetworkNumber kPivotP = new LoggedNetworkNumber("Grabber/kPivotP", 0.4);
    public static final LoggedNetworkNumber kPivotMaxVel = new LoggedNetworkNumber("Grabber/kPivotMaxVel", 150);
    public static final LoggedNetworkNumber kPivotMaxAccel = new LoggedNetworkNumber("Grabber/kPivotMaxAccel", 120);

    public static enum GrabberPosition { //TODO update positions
      DOWN(new Rotation2d(-40.0)),//-65  was at - 40 
      OUT(new Rotation2d(5.0)),
      UP(new Rotation2d(0));//70
      
      private final Rotation2d angle;

      private GrabberPosition(Rotation2d angle) {
          this.angle = angle;
      }

      public Rotation2d getAngle() {
          return angle;
      }
    }

    public static enum GrabberState {
      INTAKE(0.75),
      OUTTAKE(-0.75),
      HOLD(0.10),
      STOP(0.0);

      private final double speed;

      private GrabberState(double speed) {
          this.speed = speed;
      }

      public double getSpeed() {
          return speed;
      }
    }
  }


  public static class SwerveConstants {
    public static final Distance wheelBase = Inches.of(29);
    public static final Distance driveBaseRadius = Meters.of(Math.sqrt(wheelBase.in(Meters) * wheelBase.in(Meters) * 2) / 2);

    public static final Distance robotLength = Meters.of(0.927);
    public static final Distance robotWidth = Meters.of(0.927);

    public static final LinearVelocity kMaxVelTele = FeetPerSecond.of(19);
    public static final LinearAcceleration kMaxAccelTele = kMaxVelTele.per(Second).times(5); //idk what this should be
    public static final AngularVelocity kMaxAngularVelTele = RadiansPerSecond.of(2 * Math.PI); //idk 2 radians per second whatever
    public static final AngularAcceleration kMaxAngularAccelTele = kMaxAngularVelTele.per(Second).times(5);

    public static final LinearVelocity kMaxVelAuto = MetersPerSecond.of(1);
    public static final LinearAcceleration kMaxAccelAuto = kMaxVelAuto.per(Second).times(5);
    public static final AngularVelocity  kMaxAngularVelAuto = RadiansPerSecond.of(Math.PI);
    public static final AngularAcceleration kMaxAngularAccelAuto = kMaxAngularVelAuto.per(Second).times(5);
  }

  public static class PID {
    public static class PathPlanner {
      public static final double kPTranslation = 4.5; //4.5
      public static final double kITranslation = 0;
      public static final double kDTranslation = 0;

      public static final double kPTheta = 1.95;
      public static final double kITheta = 0.7;
      public static final double kDTheta = 0.0;
      
      public static final PIDConstants kTranslationPIDConstants = new PIDConstants(kPTranslation, kITranslation, kDTranslation);
      public static final PIDConstants kThetaPIDConstants = new PIDConstants(kPTheta, kITheta, kDTheta);
    }

    public static class PointTrack {
      public static final double kPAutoTurning = 8;
      public static final double kIAutoTurning = 0; 
      public static final double kDAutoTurning = 0;

      public static final PIDController kThetaController = new PIDController(kPAutoTurning, kIAutoTurning, kDAutoTurning);
    }

    public static class ToPoint {
      public static final LoggedNetworkNumber kPDrive = new LoggedNetworkNumber("ToPoint/kPDrive", 5);
      public static final LoggedNetworkNumber kPTheta = new LoggedNetworkNumber("ToPoint/kPTheta", 5);

      public static final LoggedNetworkNumber kDriveMaxVel = new LoggedNetworkNumber("ToPoint/kDriveMaxVel", 1);
      public static final LoggedNetworkNumber kDriveMaxAccel = new LoggedNetworkNumber("ToPoint/kDriveMaxAccel", 1);

      public static final LoggedNetworkNumber kThetaMaxVel = new LoggedNetworkNumber("ToPoint/kThetaMaxVel", Math.PI);
      public static final LoggedNetworkNumber kThetaMaxAccel = new LoggedNetworkNumber("ToPoint/kThetaMaxAccel", 2 * Math.PI);

      public static final LoggedNetworkNumber kDriveTolerance = new LoggedNetworkNumber("ToPoint/kDriveTolerance", 0.01);
      public static final LoggedNetworkNumber kThetaTolerance = new LoggedNetworkNumber("ToPoint/kThetaTolerance", 0.01);
    }
  }

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

    /**
   * Enum to represent branches of the reef.
   */
  public static enum ReefPoint {
      kCenter(new Pose2d(13, 4, new Rotation2d()), new Pose2d(4.5, 4, new Rotation2d())),
      kFarR(Utils.getOffsetRightAprilTag(10), Utils.getOffsetRightAprilTag(21)),
      kFarC(Utils.getOffsetCenterAprilTag(10), Utils.getOffsetCenterAprilTag(21)),
      kFarL(Utils.getOffsetLeftAprilTag(10), Utils.getOffsetLeftAprilTag(21)),
      kNearR(Utils.getOffsetRightAprilTag(7), Utils.getOffsetRightAprilTag(18)),
      kNearC(Utils.getOffsetCenterAprilTag(7), Utils.getOffsetCenterAprilTag(18)),
      kNearL(Utils.getOffsetLeftAprilTag(7), Utils.getOffsetLeftAprilTag(18)),
      kFarRightR(Utils.getOffsetRightAprilTag(9), Utils.getOffsetRightAprilTag(22)),
      kFarRightC(Utils.getOffsetCenterAprilTag(9), Utils.getOffsetCenterAprilTag(22)),
      kFarRightL(Utils.getOffsetLeftAprilTag(9), Utils.getOffsetLeftAprilTag(22)),
      kNearRightR(Utils.getOffsetRightAprilTag(8), Utils.getOffsetRightAprilTag(17)),
      kNearRightC(Utils.getOffsetCenterAprilTag(8), Utils.getOffsetCenterAprilTag(17)),
      kNearRightL(Utils.getOffsetLeftAprilTag(8), Utils.getOffsetLeftAprilTag(17)),
      kFarLeftR(Utils.getOffsetRightAprilTag(11), Utils.getOffsetRightAprilTag(20)),
      kFarLeftC(Utils.getOffsetCenterAprilTag(11), Utils.getOffsetCenterAprilTag(20)),
      kFarLeftL(Utils.getOffsetLeftAprilTag(11), Utils.getOffsetLeftAprilTag(20)),
      kNearLeftR(Utils.getOffsetRightAprilTag(6), Utils.getOffsetRightAprilTag(19)),
      kNearLeftC(Utils.getOffsetCenterAprilTag(6), Utils.getOffsetCenterAprilTag(19)),
      kNearLeftL(Utils.getOffsetLeftAprilTag(6), Utils.getOffsetLeftAprilTag(19));

      public final Pose2d redPose, bluePose;

      private ReefPoint(Pose2d redPose, Pose2d bluePose) {
          this.redPose = redPose;
          this.bluePose = bluePose;
      }

      public Pose2d getPose() {
          return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) ? redPose : bluePose;
      }
  }
}
