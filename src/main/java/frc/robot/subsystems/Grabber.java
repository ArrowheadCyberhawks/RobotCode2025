package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.Constants.GrabberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants.GrabberPosition;
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
        //grabberMotor2.configure(new SparkMaxConfig().inverted(false), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        // pivotMotor.configure(new SparkMaxConfig().apply(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1))), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        pivotEncoder = pivotMotor.getEncoder();

        pivotController.setGoal(getPivotAngle().getRadians());
        pivotController.setTolerance(Units.degreesToRadians(1));

        coralSensor = new TimeOfFlight(kCoralSensorPort);
        algaeSensor = new TimeOfFlight(kAlgaeSensorPort);
        coralSensor.setRangingMode(RangingMode.Short, 100);
        algaeSensor.setRangingMode(RangingMode.Short, 100);
        // pivotController.setGoal(GrabberPosition.UP.getAngle().getRotations());
        //pivotMotor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1/60.0 * 2 * Math.PI)), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }


    public void periodic() {
        updateConstants();

        // if the pivot has moved more than 30 degrees, we have left the starting position
        if(!hasLeftStartingPosition && Math.abs(pivotEncoder.getPosition()) > kPivotLimit.getRadians()) {
            hasLeftStartingPosition = true;
        }

        setPivotMotor(pivotController.calculate(pivotEncoder.getPosition()));
        setGrabberMotors(grabberState.getSpeed());

        // logging
        Logger.recordOutput(getName() + "/Has Coral", hasCoral());
        Logger.recordOutput(getName() + "/Has Algae", hasAlgae());
        Logger.recordOutput(getName() + "/Coral Range", getCoralRange().in(Meters));
        Logger.recordOutput(getName() + "/Algae Range", getAlgaeRange().in(Meters));
        Logger.recordOutput(getName() + "/Pivot Angle", getPivotAngle().getRadians());
        Logger.recordOutput(getName() + "/Pivot Target", pivotController.getGoal().position);
        Logger.recordOutput(getName() + "/Grabber State", grabberState);
    }

    private void updateConstants() {
        if (kPivotP.get() != pivotController.getP()
            || kPivotMaxVel.get() != pivotController.getConstraints().maxVelocity) {
            pivotController.setP(kPivotP.get());
            pivotController.setConstraints(new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get()));
        }
    }

    /**
     * Sets the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     */
    public void setPivotAngle(Rotation2d angle) {
        if (hasLeftStartingPosition && Math.abs(angle.getDegrees()) < kPivotLimit.getDegrees()) {
            return;
        }
        pivotController.setGoal(angle.getRadians());
    }

    
    public void setPivotPosition(GrabberPosition position) {
        setPivotAngle(position.getAngle());
    }

    public void resetPivotTarget() {
        pivotController.setGoal(getPivotAngle().getRadians());
    }

    /**
     * Sets both grabber motors to the same speed
     * @param speed
     */
    public void setGrabberMotors(double speed) {
        setGrabberMotors(speed, speed);
    }

    /**
     * Returns the current distance measured by the coral sensor.
     * @return The distance measured by the coral sensor in millimeters.
     */
    public Distance getCoralRange() {
        return Millimeters.of(coralSensor.getRange());
    }

    /**
     * Returns the current distance measured by the algae sensor.
     * @return The distance measured by the algae sensor in millimeters.
     */
    public Distance getAlgaeRange() {
        
        return Millimeters.of(algaeSensor.getRange());
    }

    /**
     * Checks if there is an object within the coral sensor's range.
     * @return whether the robot has a coral in the grabber.
     */
    public boolean hasCoral() {
        double coralRange = getCoralRange().in(Meters);
        return coralRange > 0 && coralRange < 0.1;
        //return getCoralRange().compareTo(kCoralSensorThreshold) < 0;
    }

    /**
     * Checks if there is an object within the algae sensor's range.
     * @return whether the robot has an algae in the grabber.
     */
    public boolean hasAlgae() {
        double algaeRange = getAlgaeRange().in(Meters);
        return algaeRange > 0 && algaeRange < 0.1;
    }
    
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

    /**
     * InstantCommand to simply set the grabber state.
     */
    private Command setGrabberStateCommand(GrabberState state) {
        return this.runOnce(() -> setGrabberState(state));
    }

    /**
     * Command to run the intake at full power until algae is detected, then stops.
     * @return
     */
    public Command intakeCommand() {
        return runEnd(() -> setGrabberState(GrabberState.INTAKE), () -> holdCommand().schedule()).until(this::hasAlgae);
        //make it so that run hold command after it finds algae
    }

    public Command holdCommand() {
        return runEnd(() -> setGrabberState(GrabberState.HOLD), () -> setGrabberState(GrabberState.STOP)).until(this::hasCoral);
    }

    public Command outtakeCommand() {
        return runEnd(() -> setGrabberState(GrabberState.OUTTAKE), () -> setGrabberState(GrabberState.STOP));
    }

    public Command stopIntakeCommand() {
        return setGrabberStateCommand(GrabberState.STOP);
    }
}