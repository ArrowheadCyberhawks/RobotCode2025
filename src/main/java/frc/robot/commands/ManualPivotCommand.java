package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ManualPivotCommand extends Command{

    private final Grabber grabberSubsystem;
    private final DoubleSupplier m_power;

    public ManualPivotCommand(Grabber grabberSubsystem, DoubleSupplier power){
        this.grabberSubsystem = grabberSubsystem;
        m_power = power;
        addRequirements(grabberSubsystem);
    }

    public void initialize(){
        //nothing atm
    }

    public void execute() {
        //Moves up to certain heights and applies different power values based on the enum :)
        grabberSubsystem.setPivotAngle(Rotation2d.fromRotations(grabberSubsystem.getPivotAngle() + m_power.getAsDouble()));
    }

    public boolean isFinished(){
        return false;
    }
    
}