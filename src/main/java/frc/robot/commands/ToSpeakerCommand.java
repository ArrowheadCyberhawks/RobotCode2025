package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class ToSpeakerCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    public ToSpeakerCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void execute() {
        //Figure out distance and angle to apriltag
        double xSpeed = 0;
        double ySpeed;
        double turningSpeed;
        double kPturning = 1;
        double KpDistance = 2;
        double distance =  LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ();
        double desiredDistance = 0.5;
        double distance_error = distance-desiredDistance;
        ySpeed = 0;

        //Set turning speed and y speed based off of apriltag
        turningSpeed = kPturning*LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX();
        xSpeed = LimelightHelpers.getTV("limelight") ? MathUtil.clamp(KpDistance*distance_error, -3, 3) : 0;
        //Set speeds to chassis
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        
        //Output each module states to wheels
        swerveSubsystem.driveRobotOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(xSpeed, ySpeed, turningSpeed));                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          //penis

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