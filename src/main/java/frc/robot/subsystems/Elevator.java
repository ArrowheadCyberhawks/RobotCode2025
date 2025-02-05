//don't cry ;)

package frc.robot.subsystems;

import static frc.robot.Constants.Elevator.*;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private SparkMax elevatorMotor;
    public Elevator() {
        elevatorMotor = new SparkMax(elevatorMotorID, MotorType.kBrushless);
    }

    //double monkey = 2;
    // public Command elevatorMotorUpCommand(double monkey) {
    //     double ;

    // }
}









/* 
    private final SparkMax elevatorMotor;
    private final SparkClosedLoopController elevatorController;
    }

    public void elevatorMotorMethod() {
        elevatorMotor = new SparkMax(kElevatorMotor, SparkMax.MotorType.kBrushless);
    } */


// :(