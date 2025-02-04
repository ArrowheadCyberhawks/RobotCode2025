package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AutoCommandManager {

    SendableChooser<Command> autoChooser;

    public AutoCommandManager(SwerveSubsystem swerveSubsystem) {
        configureNamedCommands(swerveSubsystem);
        //all pathplanner autos
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("SelectAuto", autoChooser);
    }

    public SendableChooser<Command> getChooser() {
        return autoChooser;
    }

    public Command getAutoManagerSelected(){
        return autoChooser.getSelected();
    }

    public void configureNamedCommands(SwerveSubsystem swerveSubsystem) { //add more when more subsystems are made
        //NamedCommands.registerCommand("intakeOn", intakeSubsystem.autoIntake());
    }
}