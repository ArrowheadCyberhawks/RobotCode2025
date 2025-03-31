package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class ManualPivotCommand extends Command{

    private final Pivot pivotSubsystem;
    private final DoubleSupplier m_power;

    public ManualPivotCommand(Pivot pivotSubsystem, DoubleSupplier power){
        this.pivotSubsystem = pivotSubsystem;
        m_power = power;
        addRequirements(pivotSubsystem);
    }

    public void initialize(){
        //nothing atm
    }

    public void execute() {
        //Moves up to certain heights and applies different power values based on the enum :)
        pivotSubsystem.setPivotAngle(Rotation2d.fromRadians(pivotSubsystem.getPivotAngle().getRadians() + m_power.getAsDouble()));
    }

    public boolean isFinished(){
        return false;
    }
    
}