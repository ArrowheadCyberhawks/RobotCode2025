package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ElevatorCommand extends Command{

    private final Elevator m_Elevator;
    private final DoubleSupplier m_power;

    public ElevatorCommand(Elevator elevatorSubsystem, DoubleSupplier power){
        m_Elevator = elevatorSubsystem;
        m_power = power;
        addRequirements(elevatorSubsystem);
    }

    public void initialize(){
        //nothing atm
    }

    public void execute() {
        //Moves up to certain heights and applies different power values based on the enum :)
        m_Elevator.setHeight(m_Elevator.getTargetHeight() + m_power.getAsDouble());
    }

    public boolean isFinished(){
        return false;
    }
    
}