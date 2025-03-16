package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private final ProfiledPIDController climbController = new ProfiledPIDController(kClimbP.get(), kClimbI.get(), 0, new Constraints(kClimbMaxVel.get(), kClimbMaxAccel.get()));                                                                                                                                                                                                                                                      //fien

    public Climber() {
        climberMotor = new SparkMax(kClimberMotorPort, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
        // climbController.setGoal(getPosition());
    }

    private void updateConstants() {
        if (kClimbP.get() != climbController.getP() // did the operator change the constants?
            || kClimbI.get() != climbController.getI()
            || kClimbMaxVel.get() != climbController.getConstraints().maxVelocity
            || kClimbMaxAccel.get() != climbController.getConstraints().maxAcceleration) {
            // FIX IT THEN
            climbController.setP(kClimbP.get());
            climbController.setI(kClimbI.get());
            climbController.setConstraints(new Constraints(kClimbMaxVel.get(), kClimbMaxAccel.get()));
        }
    }
    
    @Override
    public void periodic() {
        updateConstants();
        // climberMotor.set(climbController.calculate(climberEncoder.getPosition()));
        Logger.recordOutput(getName() + "/Position", getPosition());
    }

    public void setPosition(double position) {
        climbController.setGoal(position);
    }

    
    public double getPosition() {
        return climberEncoder.getPosition();
    }
    
    public void stop() {
        climberMotor.stopMotor();
    }

    /**
     * Sets the speed of the climber motor.
     * @param speed The speed to set the climber motor to, from -1 to 1.
     */
    public void setClimberMotor(double speed) {
        climberMotor.set(speed);
    }

    public Command runClimbCommand(Supplier<Double> power) {
        return runEnd(() -> setClimberMotor(power.get()), this::stop);
    }
}