package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.*;
import lib.frc706.cyberlib.subsystems.LimelightHelpers;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class TrackSpeakerCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, accelFunction;
    private final SlewRateLimiter xLimiter, yLimiter;

    public TrackSpeakerCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> accelFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.accelFunction = accelFunction;
        this.xLimiter = new SlewRateLimiter(15);
        this.yLimiter = new SlewRateLimiter(15);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void execute() {
        //get controller inputs
        double x = -xSpdFunction.get(); //invert because up is negative for some reason
		double y = -ySpdFunction.get(); //invert because FOC left is +y, controller right is +y
        double rot;
		double accelMultiplier = accelFunction.get();

        //deadband
		x = MathUtil.applyDeadband(x, OperatorConstants.kDriverControllerDeadband);
        y = MathUtil.applyDeadband(y, OperatorConstants.kDriverControllerDeadband);
        rot = -SwerveConstants.kPturning*LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getX() * 4.5;

        //make driving smoother
		x = Math.copySign(x * x, x);
		y = Math.copySign(y * y, y);
		x *= MathUtil.interpolate(0.15, 1, accelMultiplier);
		y *= MathUtil.interpolate(0.15, 1, accelMultiplier);
        x = xLimiter.calculate(x * OperatorConstants.kMaxVelTele);
        y = yLimiter.calculate(y * OperatorConstants.kMaxVelTele);
        
		
		swerveSubsystem.driveFieldOriented(swerveSubsystem.swerveDrive.swerveController.getRawTargetSpeeds(x, y, rot));
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