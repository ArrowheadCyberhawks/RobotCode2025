package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private final ProfiledPIDController climbController = new ProfiledPIDController(kClimbP.get(), kClimbI.get(), 0, new Constraints(kClimbMaxVel.get(), kClimbMaxAccel.get()));                                                                                                                                                                                                                                                      //fien

    public Climber() {
        climberMotor = new SparkMax(kClimberMotorPort, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();
    }

    private void updateConstants() {
        if (kClimbP.get() != climbController.getP() || kClimbMaxVel.get() != climbController.getConstraints().maxVelocity) {
            climbController.setP(kClimbP.get());
            climbController.setConstraints(new Constraints(kClimbMaxVel.get(), kClimbMaxAccel.get()));
        }
    }
    


    
    public double getPosition() {
        return climberEncoder.getPosition();
    }
    
    public void stop() {
        climberMotor.stopMotor();
    }

}