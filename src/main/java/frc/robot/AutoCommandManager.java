package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import lib.frc706.cyberlib.subsystems.SwerveSubsystem;

public class AutoCommandManager {

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public AutoCommandManager(SwerveSubsystem swerveSubsystem) {
        configureNamedCommands(swerveSubsystem);
        //all pathplanner autos
        PathPlannerAuto test = new PathPlannerAuto("Test Auto");

        autoChooser.setDefaultOption("None", null);
        //Different autos
        autoChooser.addOption("Test Auto", test);

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