package frc.robot.subsystems;

import static frc.robot.Constants.GrabberConstants.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Grabber subsystem covers the motors that manipulate the game piece
 * as well as the pivoting mechanism.
 */
public class Grabber extends SubsystemBase {
    private SparkFlex grabberMotor;
    private SparkMax pivotMotor;
    private RelativeEncoder pivotEncoder;
    private SparkClosedLoopController pivotController;
    private PIDController pivotPID = new PIDController(0.09,0, 0); //0.09

    private double setpoint = GrabberPosition.UP.getAngle().getRotations();
    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Grabber() {
        grabberMotor = new SparkFlex(kGrabberMotorPort, MotorType.kBrushless);
        pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();
        // pivotMotor.configure(new SparkMaxConfig().apply(new EncoderConfig().positionConversionFactor(1/60.0 * 2 * Math.PI)), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
     * Periodically prints the rotational values of the grabber motor and pivot motor.
     */


    public void trackGrabMot(){
        // double grabberMotorPos =  grabberMotor.getEncoder().getPosition();
        // System.out.println("Grabber Motor Position: " + grabberMotorPos);
        System.out.println("Pivot Motor Position: " + pivotEncoder.getPosition());
    }

    
    public void periodic() {
        setPivotMotor(pivotPID.calculate(pivotEncoder.getPosition(), setpoint));
        System.out.println("Pivot Motor Position: " + pivotEncoder.getPosition());
    }

    /**
     * Sets the speed of the pivot motor.
     * @param speed The percent speed to set the motor to. Should be between -1 and 1.
     */
    public void setPivotMotor(double speed) {
        pivotMotor.set(MathUtil.clamp(speed, -0.5, 0.5));
    }

    public double getPivotAngle() {
        return pivotMotor.getEncoder().getPosition();
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    /**
     * Sets the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     */
    public void setPivotAngle(Rotation2d angle) {
        System.out.println("set pivot angle");
        setpoint = angle.getRotations();
        // pivotController.setReference(10, ControlType.kPosition);
    }

    public void setPivotPosition(GrabberPosition position) {
        setpoint = position.getAngle().getRotations();
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