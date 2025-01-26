// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

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
    public static final int kDriverControllerPortUSB = 2;
    public static final int kDriverControllerPortBT = 4;
    public static final int kManipulatorControllerPortUSB = 3;
    public static final int kManipulatorControllerPortBT = 5;
    public static final double kManipulatorJoystickDeadband = 0.05;
    public static final double kDriverControllerDeadband = 0.07;
  }

  public static class SwerveConstants {
    public static final double wheelBase = Units.inchesToMeters(29);
    public static final double driveBaseRadius = Math.sqrt(wheelBase * wheelBase * 2) / 2;

     public static final double kMaxVelTele = Units.feetToMeters(15);
    public static final double kMaxAccelTele = kMaxVelTele * 1; //idk what this should be
    public static final double kMaxAngularVelTele = 2 * 2 * Math.PI; //idk 2 radians per second whatever
    public static final double kMaxAngularAccelTele = kMaxAngularVelTele * 3;
    
    public static final double kMaxVelAuto = SwerveConstants.kMaxVelTele/10;
    public static final double kMaxAccelAuto = SwerveConstants.kMaxAccelTele/10;
    public static final double  kMaxAngularVelAuto = SwerveConstants.kMaxAngularVelTele/5;
    public static final double kMaxAngularAccelAuto = SwerveConstants.kMaxAngularAccelTele/5;

    public static final Transform3d frontCamRobotToCam = new Transform3d(Units.inchesToMeters(15), Units.inchesToMeters(0), Units.inchesToMeters(6.5), new Rotation3d(Math.PI,Math.PI/6,0));
    public static final Transform3d backCamRobotToCam = new Transform3d(Units.inchesToMeters(-10), Units.inchesToMeters(-1), Units.inchesToMeters(18), new Rotation3d(0, 0, Math.PI));
   

    
  }

  public static class PID {
    public static final double kPTranslation = 1.5;//2.5
    public static final double kITranslation = 0.1;
    public static final double kDTranslation = 0.0;

    public static final double kPTheta = 5;
    public static final double kITheta = 0.1;
    public static final double kDTheta = 0.0;

    public static final PIDConstants kTranslationPIDConstants = new PIDConstants(kPTranslation, kITranslation, kDTranslation);
    public static final PIDConstants kThetaPIDConstants = new PIDConstants(kPTheta, kITheta, kDTheta);

    public static final double kPAutoTurning = 2.1;
    public static final double kIAutoTurning = 0; //0.02;//0.092505;
    public static final double kDAutoTurning = 0.1; //0.1;
  }
}
