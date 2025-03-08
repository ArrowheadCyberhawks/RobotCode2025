package frc.robot.subsystems;

import static frc.robot.Constants.GrabberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GrabberConstants.GrabberPosition;
import frc.robot.subsystems.Intake.ExtendState;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

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
    private final TimeOfFlight coralSensor, algaeSensor;
    private boolean hasLeftStartingPosition = false;

    private GrabberState grabberState = GrabberState.STOP;
    

    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Grabber() {
        grabberMotor1 = new SparkMax(kGrabberMotor1Port, MotorType.kBrushless);
        grabberMotor2 = new SparkMax(kGrabberMotor2Port, MotorType.kBrushless);
        grabberMotor2.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        pivotMotor.configure(new SparkMaxConfig().apply(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1/60.0 * 2 * Math.PI))), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotEncoder = pivotMotor.getEncoder();

        pivotController.setGoal(getPivotAngle());

        coralSensor = new TimeOfFlight(kCoralSensorPort);
        algaeSensor = new TimeOfFlight(kAlgaeSensorPort);
        coralSensor.setRangingMode(RangingMode.Short, 100);
        algaeSensor.setRangingMode(RangingMode.Short, 100);
        // pivotController.setGoal(GrabberPosition.UP.getAngle().getRotations());
        //pivotMotor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1/60.0 * 2 * Math.PI)), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    public void periodic() {
        updateConstants();
        setPivotMotor(pivotController.calculate(pivotEncoder.getPosition()));
        setGrabberMotors(grabberState.getSpeed());

        Logger.recordOutput(getName() + "/Coral Sensor", hasCoral());
        Logger.recordOutput(getName() + "/Algae Sensor", hasAlgae());
    }

    private void updateConstants() {
        if (kPivotP.get() != pivotController.getP() || kPivotMaxVel.get() != pivotController.getConstraints().maxVelocity) {
            pivotController.setP(kPivotP.get());
            pivotController.setConstraints(new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get()));
        }
    }

    public double getCoralRange() {
        return coralSensor.getRange();
    }

    public double getAlgaeRange() {
        return algaeSensor.getRange();
    }

    public boolean hasCoral() {
        return getCoralRange() < kCoralSensorThreshold;
    }

    public boolean hasAlgae() {
        return getAlgaeRange() < kAlgaeSensorThreshold;
    }

    public void setGrabberMotors(double speed) {
        setGrabberMotors(speed, speed);
    }

    /**
     * Sets the speeds of the grabber motors independently.
     * @param speed1 The percent speed to set the left motor to. Should be between -1 and 1.
     * @param speed2 The percent speed to set the right motor to. Should be between -1 and 1.
     */
    public void setGrabberMotors(double speed1, double speed2) {
        grabberMotor1.set(speed1);
        grabberMotor2.set(-speed2);
    }

    /**
     * Stops both grabber motors.
     */
    public void stopGrabberMotors() {
        grabberMotor1.stopMotor();
        grabberMotor2.stopMotor();
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

    public Command intakeCommand() {
        return setGrabberStateCommand(GrabberState.INTAKE).repeatedly().until(this::hasAlgae).finallyDo(() -> holdCommand().schedule());
        //make it so that run hold command after it finds algae
    }

    public Command holdCommand() {
        return setGrabberStateCommand(GrabberState.HOLD).repeatedly().until(this::hasCoral).finallyDo(() -> stopIntakeCommand().schedule());
    }

    public Command outtakeCommand() {
        return setGrabberStateCommand(GrabberState.OUTTAKE).finallyDo(() -> stopIntakeCommand().schedule());
    }

    public Command stopIntakeCommand() {
        return setGrabberStateCommand(GrabberState.STOP);
    }

    private Command setGrabberStateCommand(GrabberState state) {
        return new InstantCommand(() -> setGrabberState(state));
    }
}