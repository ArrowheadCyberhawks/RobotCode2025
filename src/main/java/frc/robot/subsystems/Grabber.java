package frc.robot.subsystems;

import static frc.robot.Constants.GrabberConstants.*;
import frc.robot.Constants.SensorConstants;
import com.revrobotics.spark.SparkFlex;
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
    private final SparkFlex grabberMotor;
    private final SparkMax pivotMotor;
    private final RelativeEncoder pivotEncoder;
    private final ProfiledPIDController pivotController = new ProfiledPIDController(0.4, 0, 0, new Constraints(kPivotMaxVel, kPivotMaxAccel)); //0.09
    //private final TimeOfFlight grabberSensor;


    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Grabber() {
        grabberMotor = new SparkFlex(kGrabberMotorPort, MotorType.kBrushless);
        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        pivotEncoder = pivotMotor.getEncoder();
        pivotController.setGoal(getPivotAngle());
        //grabberSensor = new TimeOfFlight(SensorConstants.kTimeOfFlightPort);
        // pivotController.setGoal(GrabberPosition.UP.getAngle().getRotations());
        //pivotMotor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1/60.0 * 2 * Math.PI)), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    public void periodic() {
        setPivotMotor(pivotController.calculate(pivotEncoder.getPosition()));
    }


    /*public double getSensorDistance() {
        return grabberSensor.getRange();
    }*/

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
    public Command temp1(GrabberPosition position) {
        System.out.println("Left Bumper Pressed!!!");
        return runOnce(() -> setPivotPosition(position));
    }
    /**
     * Runs the grabber motor at a given speed.
     * @param speed The speed to run the grabber motor at. Should be between -1 and 1.
     * @return A command that runs the grabber motor at the given speed.
     */
    public Command runGrabberCommand(double speed) {
        return new RunCommand(() -> setGrabberMotor(speed)).finallyDo(() -> stopGrabberMotor());
    }

    public Command runPivotCommand(double speed) {
        return runEnd(() -> setPivotMotor(speed), () -> stopPivotMotor());
    }

    public Command stopGrabberCommand() {
        return runOnce(() -> stopGrabberMotor());
    }
}