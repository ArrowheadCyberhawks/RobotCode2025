package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.GrabberConstants.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Grabber subsystem covers the motors that manipulate the game piece
 * as well as the pivoting mechanism.
 */
public class Grabber extends SubsystemBase {
    private SparkFlex grabberMotor;
    private SparkMax pivotMotor, kickerMotor;
    private SparkClosedLoopController pivotController;

    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Grabber() {
        grabberMotor = new SparkFlex(kGrabberMotorPort, MotorType.kBrushless);
        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        kickerMotor = new SparkMax(kKickerMotorPort, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
    }

    /**
     * Sets the speed of the grabber motor.
     * @param speed The percent speed to set the motor to. Should be between -1 and 1.
     */
    public void setGrabberMotor(double speed) {
        grabberMotor.set(speed);
    }

    /**
     * Stops the grabber motor.
     */
    public void stopGrabberMotor() {
        grabberMotor.stopMotor();
    }

    /**
     * Sets the speed of the pivot motor.
     * @param speed The percent speed to set the motor to. Should be between -1 and 1.
     */
    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    /**
     * Sets the speed of the kicker motor.
     * @param speed The percent speed to set the motor to. Should be between -1 and 1.
     */
    public void setKickerMotor(double speed) {
        kickerMotor.set(speed);
    }

    /**
     * Stops the kicker motor.
     */
    public void stopKickerMotor() {
        kickerMotor.stopMotor();
    }

    /**
     * Sets the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     */
    public void setPivotAngle(Rotation2d angle) {
        pivotController.setReference(angle.getRotations(), ControlType.kMAXMotionPositionControl);
    }

    /**
     * Command to set the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     * @return A command that sets the pivot angle of the grabber.
     */
    public Command setPivotAngleCommand(Rotation2d angle) {
        return runOnce(() -> setPivotAngle(angle));
    }

    /**
     * Runs the grabber motor at a given speed.
     * @param speed The speed to run the grabber motor at. Should be between -1 and 1.
     * @return A command that runs the grabber motor at the given speed.
     */
    public Command runGrabberCommand(double speed) {
        return runEnd(() -> setGrabberMotor(speed), () -> stopGrabberMotor());
    }

    /**
     * Ejects the game piece using the kicker motor.
     * @return A command that runs the kicker motor at full speed for 1 second.
     */
    public Command kickCommand() {
        return runEnd(() -> setKickerMotor(1), () -> stopKickerMotor()).withTimeout(Seconds.of(1));
    }
}