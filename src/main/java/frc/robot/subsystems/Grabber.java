package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static frc.robot.constants.Constants.GrabberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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
import frc.robot.subsystems.LEDSubsystem.LEDState;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;


/**
 * The Grabber subsystem covers the motors that manipulate the game piece
 * as well as the pivoting mechanism.
 */
public class Grabber extends SubsystemBase {
    private final SparkMax grabberMotor1; // left motor
    private final SparkMax grabberMotor2; // right motor
    private final SparkMaxConfig grabberMotor1Config;
    private final SparkMaxConfig grabberMotor2Config;
    private final TimeOfFlight coralSensor, algaeSensor;

    private GrabberState grabberState = GrabberState.STOP;
    

    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Grabber() {
        grabberMotor1 = new SparkMax(kGrabberMotor1Port, MotorType.kBrushless);
        grabberMotor2 = new SparkMax(kGrabberMotor2Port, MotorType.kBrushless);
        grabberMotor1Config = new SparkMaxConfig();
        grabberMotor2Config = new SparkMaxConfig();
        grabberMotor1Config.smartCurrentLimit(5);
        grabberMotor2Config.smartCurrentLimit(5);
        grabberMotor1Config.inverted(true);
 

        coralSensor = new TimeOfFlight(kCoralSensorPort);
        algaeSensor = new TimeOfFlight(kAlgaeSensorPort);
        coralSensor.setRangingMode(RangingMode.Short, 100);
        algaeSensor.setRangingMode(RangingMode.Short, 100);
    }


    public void periodic() {
        //in theory we could set this to two different speeds, but we'll see
        setGrabberMotors(grabberState.getSpeed(), grabberState.getSpeed());
        Logger.recordOutput(getName() + "/Has Coral", hasCoral());
        Logger.recordOutput(getName() + "/Has Algae", hasAlgae());

        //Controling LEDS

        // if(hasAlgae() || hasCoral()) {
        //     LEDSubsystem.ledState = LEDState.IN;
        // }
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

    public void setCurrentLimit(int amps) {
        grabberMotor1Config.smartCurrentLimit(amps);
        grabberMotor2Config.smartCurrentLimit(amps);
        grabberMotor1.configure(grabberMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        grabberMotor2.configure(grabberMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        stopGrabberMotors();
        setCurrentLimit(5);
        return Commands.runEnd(() -> setGrabberState(GrabberState.INTAKE), stopIntakeCommand()::schedule).until(this::hasCoral);
        //make it so that run hold command after it finds algae
    }

    public Command holdCommand() {
        setCurrentLimit(5);
        return Commands.runEnd(() -> setGrabberState(GrabberState.HOLD), () -> setGrabberState(GrabberState.STOP)).until(this::hasCoral);
    }

    public Command outtakeCommand() {
        stopGrabberMotors();
        setCurrentLimit(40);
        return Commands.runEnd(() -> setGrabberState(GrabberState.OUTTAKE), () -> setGrabberState(GrabberState.STOP)).onlyWhile(this::hasAlgae);
    }

    public Command stopIntakeCommand() {
        return setGrabberStateCommand(GrabberState.STOP);
    }
    public Command getGamePieceCommand() {
        return (hasAlgae()) ? outtakeCommand() : intakeCommand();
    }
}