package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private final ProfiledPIDController elevatorController = new ProfiledPIDController(kElevatorP.get(), 0, 0, new Constraints(kElevatorMaxVel.get(), kElevatorMaxAccel.get()));

    public Elevator() {
        elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorController.setGoal(getHeight().in(Meters));
        elevatorController.setTolerance(0.01);
    }

    @Override
    public void periodic() {
        // updateConstants();
        elevatorMotor.set(elevatorController.calculate(elevatorEncoder.getPosition()));
        SmartDashboard.putNumber("Elevator Height", elevatorEncoder.getPosition());
        Logger.recordOutput(getName() + "/Height", getHeight().in(Meters));
    }

    private void updateConstants() {
        if (kElevatorP.get() != elevatorController.getP()
            || kElevatorMaxVel.get() != elevatorController.getConstraints().maxVelocity
            || kElevatorMaxAccel.get() != elevatorController.getConstraints().maxAcceleration) {
            elevatorController.setP(kElevatorP.get());
            elevatorController.setConstraints(new Constraints(kElevatorMaxVel.get(), kElevatorMaxAccel.get()));
        }
    }

    public Distance getHeight() {
        return Meters.of(elevatorEncoder.getPosition());
    }

    /**
     * @return Returns the target encoder position of the elevator.
     */
    public double getTargetPosition() {
        return elevatorController.getGoal().position;
    }

    public boolean atTarget() {
        return elevatorController.atGoal();
    }

    /**
     * Set the elevator PID to a specific position.
     * @param height The position to set the elevator to IN METERS.
     */
    public void setHeight(double height) {
        if (height < 0) {
            height = 0;
        }
        elevatorController.setGoal(height);
    }

    /**
     * @return A command to bring the elevator to the height of the L1 branch.
     */
    public Command L1() {
        return setLevelCommand(ElevatorLevel.L1);
    }

    /**
     * @return A command to bring the elevator to the height of the L2 branch.
     */
    public Command L2() {
        return setLevelCommand(ElevatorLevel.L2);
    }

    /**
     * 
     * @return A command to bring the elevator to the height of the L3 branch.
     */
    public Command L3() {
        return setLevelCommand(ElevatorLevel.L3);
    }

    /**
     * 
     * @return A command to bring the elevator to the height of the L4 branch.
     */
    public Command L4() {
        return setLevelCommand(ElevatorLevel.L4);
    }
    
    /**
     * @param level The RELATIVE encoder value the elevator motor will move up to.
     * @return Returns a command to move the elevator to a certain encoder position.
     */
    public Command setLevelCommand(ElevatorLevel level) {
        return runOnce(() -> setHeight(level.getHeight()));
    }
    
    /**
     * @param power The power that the elevator motor will run at.
     * @return Returns a command to move the elevator up/down at a certain power.
     */
    public Command runPowerCommand(Supplier<Double> power) {
        return run(() -> setHeight(getTargetPosition() + power.get()));
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