//don't cry ;)

package frc.robot.subsystems;

import static frc.robot.Constants.Elevator.*;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ElevatorCommand;


public class Elevator extends SubsystemBase {
    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorPID;
    private TrapezoidProfile trapezoidProfile;
    
    private ControlState controlState = ControlState.MANUAL;
    private ElevatorLevel targetLevel = ElevatorLevel.L1;
    Distance targetHeight = targetLevel.getHeight();

    public Elevator() {
        elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorPID = elevatorMotor.getClosedLoopController();
        trapezoidProfile = new TrapezoidProfile(new Constraints(elevatorMotor.configAccessor.closedLoop.maxMotion.getMaxVelocity(), elevatorMotor.configAccessor.closedLoop.maxMotion.getMaxAcceleration()));
    }    

    @Override
    public void periodic() {
        if (controlState == ControlState.AUTO) {
            setPosition(trapezoidProfile.calculate(0.02, getState(), new State(targetLevel.getHeight(), 0)).position);
        }
    }

    public State getState() {
        return new State(elevatorEncoder.getPosition(), elevatorEncoder.getVelocity());
    }

    //Enum that controls how the driver moves the slides


    /**
     * @return Returns the target encoder position of the elevator.
     */
    public double getTargetPosition() {
        return targetHeight;
    }

    /**
     * Set the elevator PID to a specific position.
     * @param position The position to set the elevator to IN ROTATIONS.
     */
    public void setPosition(double position) {
        elevatorPID.setReference(position, ControlType.kMAXMotionPositionControl);

    }

    /**
     * @return A command to bring the elevator to the height of the L1 branch.
     */
    public Command L1() {
        return runOnce(() -> targetLevel = ElevatorLevel.L1);
    }

    /**
     * @return A command to bring the elevator to the height of the L2 branch.
     */
    public Command L2() {
        return runOnce(() -> targetLevel = ElevatorLevel.L2);
    }

    /**
     * 
     * @return A command to bring the elevator to the height of the L3 branch.
     */
    public Command L3() {
        return runOnce(() -> targetLevel = ElevatorLevel.L3);
    }

    /**
     * 
     * @return A command to bring the elevator to the height of the L4 branch.
     */
    public Command L4() {
        return runOnce(() -> targetLevel = ElevatorLevel.L4);
    }
    
    /**
     * @param position The RELATIVE encoder value the elevator motor will move up to.
     * @return Returns a command to move the elevator to a certain encoder position.
     */
    public Command setPositionCommand(ElevatorLevel position) {
        return runOnce(() -> this.targetLevel = position);
    }
    
    /**
     * @param power The power that the elevator motor will run at.
     * @return Returns a command to move the elevator up/down at a certain power.
     */
    public Command runPowerCommand(Supplier<Double> power) {
        return run(() -> setPosition(getTargetPosition() + power.get()));
    }

    /**
     * @param state
     * @return
     */
    public Command setControlStateCommand(ControlState state) {
        if (state == ControlState.AUTO) {
            return runOnce(() -> controlState = state).andThen(setPositionCommand(this.targetPosition));
        } else {
            return new ElevatorCommand(this, null);
        }
    }
    
    /**
     * @return Returns a command that resets the elevator motor.
     */
    public Command resetElevatorEncoders() {
        return runOnce(() -> elevatorEncoder.setPosition(0));
    }

    /**
     * Stops the elevator motor.
     */
    public void stop() {
        elevatorMotor.stopMotor();
    }
}









/* 
    private final SparkMax elevatorMotor;
    private final SparkClosedLoopController elevatorController;
    }

    public void elevatorMotorMethod() {
        elevatorMotor = new SparkMax(kElevatorMotor, SparkMax.MotorType.kBrushless);
    } */


// :(