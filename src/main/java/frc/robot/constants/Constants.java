// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.commons.TagUtils;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */



public final class Constants {

  public static final boolean tuningMode = false; //change this to tune

  public static class IOConstants {
    public static final int kDriverControllerPortUSB = 0;
    public static final int kManipulatorControllerPortUSB = 1;
    public static final int kDriverControllerPortBT = 2;
    public static final int kManipulatorControllerPortBT = 3;

    public static final int kKeypadPort = 5;

    public static final double kManipulatorJoystickDeadband = 0.05;
    public static final double kDriverControllerDeadband = 0.04;

  }

  // public static class IntakeConstants {
  //   public static final int kIntakeMotorPort = 13;
  //   public static final int kExtendMotorPort = 14;

  //   public static enum intakeState {
  //       In(0.0),
  //       Out(1.0);
  //   }
  // }

  public static class ClimberConstants {
    public static final int kClimberMotorPort = 14; // change later 

    public static final LoggedNetworkNumber kClimbP = new LoggedNetworkNumber("Climb/kPivotP", 0.1); //change later 
    public static final LoggedNetworkNumber kClimbI = new LoggedNetworkNumber("Climb/kPivotI", 0);

    public static final LoggedNetworkNumber kClimbOutAngle = new LoggedNetworkNumber("Climb/kClimbOutAngle", -30);
    public static final LoggedNetworkNumber kClimbInAngle = new LoggedNetworkNumber("Climb/kClimbInAngle", 330);
    
    public static final LoggedNetworkNumber kClimbMaxVel = new LoggedNetworkNumber("Climb/kElevatorMaxVel", 100);
    public static final LoggedNetworkNumber kClimbMaxAccel = new LoggedNetworkNumber("Climb/kElevatorMaxAccel", 100);

  }

  public static class ElevatorConstants {
    public static final LoggedNetworkNumber kElevatorP = new LoggedNetworkNumber("Elevator/kElevatorP", 3.5);
    public static final LoggedNetworkNumber kElevatorMaxVel = new LoggedNetworkNumber("Elevator/kElevatorMaxVel", 6);
    public static final LoggedNetworkNumber kElevatorMaxAccel = new LoggedNetworkNumber("Elevator/kElevatorMaxAccel", 10);

    public static final int elevatorMotorID = 9;

    public static final double maxHeight = 1.65;

    public static enum ElevatorLevel {
      LO(0.0),
      L1(0.444),
      L2(0.267), //0.269 
      L3(0.66), 
      L4(1.327 ), //1.354
      HI(1.65),
      HUMAN(0.930),
      CLEAR(1.1),
      ALG3(0.849),
      ALG2(0.46);

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

    public static final int kPivotEncoderId = 55;

    public static final double kMaxPivotPower = 0.7;

    public static final Rotation2d kPivotLimit = Rotation2d.fromDegrees(0);

    public static final Distance kArmLength = Inches.of(30); //real length is like 22 but this is accounting for the size of the grabber

    public static final Distance kCoralSensorThreshold = Centimeters.of(10);
    public static final Distance kAlgaeSensorThreshold = Centimeters.of(2);

    public static final LoggedNetworkNumber kPivotP = new LoggedNetworkNumber("Grabber/kPivotP", 7.5);//0.7
    public static final LoggedNetworkNumber kPivotI = new LoggedNetworkNumber("Grabber/kPivotI", 0.0);
    public static final LoggedNetworkNumber kPivotD = new LoggedNetworkNumber("Grabber/kPivotD", 0.75);
    public static final LoggedNetworkNumber kPivotMaxVel = new LoggedNetworkNumber("Grabber/kPivotMaxVel", 3);
    public static final LoggedNetworkNumber kPivotMaxAccel = new LoggedNetworkNumber("Grabber/kPivotMaxAccel", 10);

    public static final LoggedNetworkNumber kPivotUpP = new LoggedNetworkNumber("Grabber/kPivotUpP", 0.3);

    public static final LoggedNetworkNumber kPivotS = new LoggedNetworkNumber("Grabber/kPivotS", 0.0);
    public static final LoggedNetworkNumber kPivotG = new LoggedNetworkNumber("Grabber/kPivotG", 0.4);
    public static final LoggedNetworkNumber kPivotV = new LoggedNetworkNumber("Grabber/kPivotV", 0.15);
    public static final LoggedNetworkNumber kPivotA = new LoggedNetworkNumber("Grabber/kPivotA", 0.04);

    public static final double grabberOffset = Math.PI/2 + 0.21;
    public static enum PivotPosition { //TODO update positions
      //DOWN(Rotation2d.kPi),//-65  was at - 40 
      
      OUT(Rotation2d.fromDegrees(5.0 + grabberOffset)),
      PLACE(Rotation2d.fromRadians(1.1 + grabberOffset)),
      L1(Rotation2d.fromRadians(Math.PI + grabberOffset)),
      L4(Rotation2d.fromRadians(0.96 + grabberOffset)),
      HUMAN(Rotation2d.fromRadians(3.902 + grabberOffset)),
      ZERO(Rotation2d.kZero),

      ALGPICK(Rotation2d.fromRadians(0.622 + grabberOffset)),
      ALGREEF(Rotation2d.fromRadians(1.125 + grabberOffset)),
      HI(Rotation2d.fromRadians(1.762 + grabberOffset));//70
      
      private final Rotation2d angle;

      private PivotPosition(Rotation2d angle) {
          this.angle = angle;
          //put grabberoffset here next time
      }

      public Rotation2d getAngle() {
          return angle;
         //put grabberoffset here next time

      }
    }

    public static enum GrabberState {
      INTAKE(0.6),
      OUTTAKE(-0.90),
      HOLD(0.15),
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

    //TODO Tune to be slower
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
      public static final double kPTranslation = 5.5; //2.8
      public static final double kITranslation = 0.2; //0.2
      public static final double kDTranslation = 0.0; //0.2

      public static final double kPTheta = 5; //2.95
      public static final double kITheta = 0.1; //0.7
      public static final double kDTheta = 0.1;
      
      public static final PIDConstants kTranslationPIDConstants = new PIDConstants(kPTranslation, kITranslation, kDTranslation);
      public static final PIDConstants kThetaPIDConstants = new PIDConstants(kPTheta, kITheta, kDTheta);
    }

    public static class Auto {
      public static final double kPTranslation = 8.45; //6.5
      public static final double kITranslation = 0.3;
      public static final double kDTranslation = 0.01;

      public static final double kPTheta = 6;
      public static final double kITheta = 0.1;
      public static final double kDTheta = 0.005;
      
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
      // public static final LoggedNetworkNumber kPDrive = new LoggedNetworkNumber("ToPoint/kPDrive", 5);
      // public static final LoggedNetworkNumber kPTheta = new LoggedNetworkNumber("ToPoint/kPTheta", 5);
    
      // public static final LoggedNetworkNumber kIDrive = new LoggedNetworkNumber("ToPoint/kIDrive", 0.001);

      // public static final LoggedNetworkNumber kDriveMaxVel = new LoggedNetworkNumber("ToPoint/kDriveMaxVel", 1);
      // public static final LoggedNetworkNumber kDriveMaxAccel = new LoggedNetworkNumber("ToPoint/kDriveMaxAccel", 1);

      // public static final LoggedNetworkNumber kThetaMaxVel = new LoggedNetworkNumber("ToPoint/kThetaMaxVel", Math.PI);
      // public static final LoggedNetworkNumber kThetaMaxAccel = new LoggedNetworkNumber("ToPoint/kThetaMaxAccel", 2 * Math.PI);

      // public static final LoggedNetworkNumber kDriveTolerance = new LoggedNetworkNumber("ToPoint/kDriveTolerance", 0.005);
      // public static final LoggedNetworkNumber kThetaTolerance = new LoggedNetworkNumber("ToPoint/kThetaTolerance", 0.001);
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
    //TODO Change all "get offset" to use FieldConstants instead of parsing the json data
      kFarR(TagUtils.getOffsetRightAprilTag(10), TagUtils.getOffsetRightAprilTag(21), 10, 21),
      kFarC(TagUtils.getOffsetCenterAprilTag(10), TagUtils.getOffsetCenterAprilTag(21), 10, 21),
      kFarL(TagUtils.getOffsetLeftAprilTag(10), TagUtils.getOffsetLeftAprilTag(21), 10, 21),
      kNearR(TagUtils.getOffsetRightAprilTag(7), TagUtils.getOffsetRightAprilTag(18), 7, 18),
      kNearC(TagUtils.getOffsetCenterAprilTag(7), TagUtils.getOffsetCenterAprilTag(18), 7, 18),
      kNearL(TagUtils.getOffsetLeftAprilTag(7), TagUtils.getOffsetLeftAprilTag(18), 7, 18),
      kFarRightR(TagUtils.getOffsetRightAprilTag(9), TagUtils.getOffsetRightAprilTag(22), 9 ,22),
      kFarRightC(TagUtils.getOffsetCenterAprilTag(9), TagUtils.getOffsetCenterAprilTag(22),9 ,22),
      kFarRightL(TagUtils.getOffsetLeftAprilTag(9), TagUtils.getOffsetLeftAprilTag(22), 9 ,22),
      kNearRightR(TagUtils.getOffsetRightAprilTag(8), TagUtils.getOffsetRightAprilTag(17), 8 ,17),
      kNearRightC(TagUtils.getOffsetCenterAprilTag(8), TagUtils.getOffsetCenterAprilTag(17), 8 ,17),
      kNearRightL(TagUtils.getOffsetLeftAprilTag(8), TagUtils.getOffsetLeftAprilTag(17), 8 ,17),
      kFarLeftR(TagUtils.getOffsetRightAprilTag(11), TagUtils.getOffsetRightAprilTag(20), 11, 20),
      kFarLeftC(TagUtils.getOffsetCenterAprilTag(11), TagUtils.getOffsetCenterAprilTag(20), 11, 20),
      kFarLeftL(TagUtils.getOffsetLeftAprilTag(11), TagUtils.getOffsetLeftAprilTag(20), 11, 20),
      kNearLeftR(TagUtils.getOffsetRightAprilTag(6), TagUtils.getOffsetRightAprilTag(19), 6, 19),
      kNearLeftC(TagUtils.getOffsetCenterAprilTag(6), TagUtils.getOffsetCenterAprilTag(19), 6, 19),
      kNearLeftL(TagUtils.getOffsetLeftAprilTag(6), TagUtils.getOffsetLeftAprilTag(19), 6, 19),
      kCenter(new Pose2d(13, 4, new Rotation2d()), new Pose2d(4.5, 4, new Rotation2d()), -1, -1);

      public final Pose2d redPose, bluePose;
      public final int redTag, blueTag;

      private ReefPoint(Pose2d redPose, Pose2d bluePose, int redTag, int blueTag) {
          this.redPose = redPose;
          this.bluePose = bluePose;
          this.redTag = redTag;
          this.blueTag = blueTag;
      }

      public Pose2d getPose() {
          return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) ? redPose : bluePose;
      }

      public int getTag() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red) ? redTag : blueTag;
      }
  }

  public static final class LEDConstants {
    public static final int LED_PWM = 0; //port of LEDs on robot
  }

  public static class CameraConstants {
    public static class cam0 {
      public static final String name = "cam0";
      public static final Translation3d translation = new Translation3d(Inches.of(-1.75), Inches.of(-3.1875), Inches.of(5));
      public static final Rotation3d rotation = new Rotation3d(0, Units.degreesToRadians(-20), 0);
      public static final Transform3d offset = new Transform3d(translation, rotation);
    }
    public static class cam1 {
      public static final String name = "cam1";
      public static final Translation3d translation = new Translation3d(Inches.of(-0.75), Inches.of(-12.75), Inches.of(10.25));
      public static final Rotation3d rotation = new Rotation3d(Units.degreesToRadians(0),Units.degreesToRadians(0), Units.degreesToRadians(-5));
      public static final Transform3d offset = new Transform3d(translation, rotation);
    }

    /*cam2 = new PhotonCameraWrapper("cam2", new Transform3d(new Translation3d(Inches.of(12.625), Inches.of(4.75), Inches.of(30.5)), new Rotation3d(0,0, 0))); // front forwards
    cam3 = new PhotonCameraWrapper("cam3", new Transform3d(new Translation3d(Inches.of(13.625), Inches.of(2.75), Inches.of(30.5)), new Rotation3d(Units.degreesToRadians(-8.3),0, -Math.PI/2))); // front right
    cam4 = new PhotonCameraWrapper("cam4", new Transform3d(new Translation3d(Inches.of(-13.625), Inches.of(2.75), Inches.of(29)), new Rotation3d(Units.degreesToRadians(8.3),0, -Math.PI/2))); // rear right
    cam5 = new PhotonCameraWrapper("cam5", new Transform3d(new Translation3d(Inches.of(-12.625), Inches.of(4.75), Inches.of(29.75)), new Rotation3d(0,0, Math.PI))); // rear backwards
    cam6 = new PhotonCameraWrapper("cam6", new Transform3d(new Translation3d(Inches.of(-13.625), Inches.of(6.75), Inches.of(30.25)), new Rotation3d(Units.degreesToRadians(8.3),0, Math.PI/2))); // rear left*/

  }

}
