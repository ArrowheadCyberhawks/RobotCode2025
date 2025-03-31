package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.constants.Constants.GrabberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
import frc.robot.constants.Constants.GrabberConstants.GrabberState;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;

/**
 * The Grabber subsystem covers the motors that manipulate the game piece
 * as well as the pivoting mechanism.
 */
public class Pivot extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final ProfiledPIDController pivotController = new ProfiledPIDController(kPivotP.get(), 0, 0, new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get())); //0.09

    private GrabberState grabberState = GrabberState.STOP;
    

    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Pivot() {
        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();

        pivotController.setGoal(getPivotAngle().getRadians());
        pivotController.setTolerance(Units.degreesToRadians(3));
    }


    public void periodic() {
        // updateConstants();
        setPivotMotor(MathUtil.clamp(pivotController.calculate(pivotEncoder.getPosition()), -kMaxPivotPower, kMaxPivotPower));

        SmartDashboard.putNumber("Pivot Angle", getPivotAngle().getRadians());
        SmartDashboard.putNumber("Pivot Target", pivotController.getGoal().position);
        // logging
        Logger.recordOutput(getName() + "/Pivot Angle", getPivotAngle().getRadians());
        Logger.recordOutput(getName() + "/Pivot Target", pivotController.getGoal().position);
    }

    // private void updateConstants() {
    //     if (kPivotP.get() != pivotController.getP()
    //         || kPivotMaxVel.get() != pivotController.getConstraints().maxVelocity) {
    //         pivotController.setP(kPivotP.get());
    //         pivotController.setConstraints(new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get()));
    //     }
    // }

    /**
     * Sets the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is up, positive is clockwise looking at the front of the r.
     */
    public void setPivotAngle(Rotation2d angle) {
        if (Math.abs(angle.getDegrees()) < kPivotLimit.getDegrees()) {
            return;
        }
        pivotController.setGoal(angle.getRadians());
    }
    
    public void setPivotPosition(PivotPosition position) {
        setPivotAngle(position.getAngle());
    }

    public void resetPivotTarget() {
        pivotController.setGoal(getPivotAngle().getRadians());
    }
    
    public void setGrabberState(GrabberState state) {
        grabberState = state;
    }

    /**
     * Sets the speed of the pivot motor.
     * @param speed The percent speed to set the motor to. Should be between -1 and 1.
     */
    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }
    
    /**
     * Returns the position of the pivot motor.
     * @return A Rotation2d representing the position of the pivot motor encoder.
     */
    public Rotation2d getPivotAngle() {
        return new Rotation2d(pivotMotor.getEncoder().getPosition());
    }

    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    public boolean atPivotTarget() {
        return pivotController.atGoal();
    }

    public void resetPivotAngle(Rotation2d angle) {
        pivotEncoder.setPosition(angle.getRadians());
    }

    /**
     * Command to set the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     * @return A command that sets the pivot angle of the grabber.
     */
    public Command setPivotAngleCommand(Rotation2d angle) {
        return runOnce(() -> setPivotAngle(angle));
    }

    public Command setPivotPositionCommand(PivotPosition position) {
        return runOnce(() -> setPivotPosition(position));
    }

    public Command runPivotCommand(double speed) {
        return runEnd(() -> setPivotMotor(speed), () -> stopPivotMotor());
    }

    /**
     * InstantCommand to simply set the grabber state.
     */
    private Command setGrabberStateCommand(GrabberState state) {
        return this.runOnce(() -> setGrabberState(state));
    }

    public Command stopIntakeCommand() {
        return setGrabberStateCommand(GrabberState.STOP);
    }
}