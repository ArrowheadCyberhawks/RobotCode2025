package frc.robot.subsystems;

import static frc.robot.Constants.GrabberConstants.*;
import frc.robot.Constants.SensorConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.playingwithfusion.TimeOfFlight;

/**
 * The Grabber subsystem covers the motors that manipulate the game piece
 * as well as the pivoting mechanism.
 */
public class Grabber extends SubsystemBase {
    private final SparkMax grabberMotor1; // left motor
    private final SparkMax grabberMotor2; // right motor
    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final ProfiledPIDController pivotController = new ProfiledPIDController(kPivotP.get(), 0, 0, new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get())); //0.09
    //private final TimeOfFlight grabberSensor;


    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Grabber() {
        grabberMotor1 = new SparkMax(kGrabberMotor1Port, MotorType.kBrushless);
        grabberMotor2 = new SparkMax(kGrabberMotor2Port, MotorType.kBrushless);
        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotController.setGoal(getPivotAngle());
        //grabberSensor = new TimeOfFlight(SensorConstants.kTimeOfFlightPort);
        // pivotController.setGoal(GrabberPosition.UP.getAngle().getRotations());
        //pivotMotor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1/60.0 * 2 * Math.PI)), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    public void periodic() {
        updateConstants();
        setPivotMotor(pivotController.calculate(pivotEncoder.getPosition()));
    }

    private void updateConstants() {
        if (kPivotP.get() != pivotController.getP() || kPivotMaxVel.get() != pivotController.getConstraints().maxVelocity) {
            pivotController.setP(kPivotP.get());
            pivotController.setConstraints(new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get()));
        }
    }


    /*public double getSensorDistance() {
        return grabberSensor.getRange();
    }*/

    /**
     * Sets the speeds of the grabber motors independently.
     * @param speed1 The percent speed to set the left motor to. Should be between -1 and 1.
     * @param speed2 The percent speed to set the right motor to. Should be between -1 and 1.
     */
    public void setGrabberMotors(double speed1, double speed2) {
        grabberMotor1.set(speed1);
        grabberMotor2.set(speed2);
    }

    /**
     * Stops both grabber motors.
     */
    public void stopGrabberMotors() {
        grabberMotor1.stopMotor();
        grabberMotor2.stopMotor();
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
     * @return The position of the encoder on the pivot motor.
     */
    public double getPivotAngle() {
        return pivotMotor.getEncoder().getPosition();
    }

    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    /**
     * Sets the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     */
    public void setPivotAngle(Rotation2d angle) {
        pivotController.setGoal(angle.getRotations());
    }

    
    public void setPivotPosition(GrabberPosition position) {
        setPivotAngle(position.getAngle());
    }

    /**
     * Command to set the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     * @return A command that sets the pivot angle of the grabber.
     */
    public Command setPivotAngleCommand(Rotation2d angle) {
        return runOnce(() -> setPivotAngle(angle));
    }

    public Command setPivotPositionCommand(GrabberPosition position) {
        return runOnce(() -> setPivotPosition(position));
    }

    /**
     * Runs each grabber motor at a given speed.
     * @param speed1 The speed to run the left grabber motor at. Should be between -1 and 1.
     * @param speed2 The speed to run the right grabber motor at. Should be between -1 and 1.
     * @return A command that runs the grabber motors at the given speed.
     */
    public Command runGrabberCommand(double speed1, double speed2) {
        return new RunCommand(() -> setGrabberMotors(speed1, speed2)).finallyDo(() -> stopGrabberMotors());
    }

    /**
     * Runs both grabber motors at the same speed.
     * @param speed The speed to run the grabber motors at. Should be between -1 and 1.
     * @return A command that runs the grabber motors at the given speed.
     */
    public Command runGrabberCommand(double speed) {
        return runGrabberCommand(speed, speed);
    }

    public Command runPivotCommand(double speed) {
        return runEnd(() -> setPivotMotor(speed), () -> stopPivotMotor());
    }

    public Command stopGrabberCommand() {
        return runOnce(() -> stopGrabberMotors());
    }
}