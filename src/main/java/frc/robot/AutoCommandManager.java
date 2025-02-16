package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import lib.frc706.cyberlib.subsystems.*;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.GrabberConstants.GrabberPosition;
import frc.robot.subsystems.*;


public class AutoCommandManager {

    SendableChooser<Command> autoChooser;

    public AutoCommandManager(SwerveSubsystem swerveSubsystem, Intake intakeSubsystem, Elevator elevatorSubsystem, Grabber grabberSubsystem) {
        configureNamedCommands(swerveSubsystem, intakeSubsystem, elevatorSubsystem, grabberSubsystem);
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

    public void configureNamedCommands(SwerveSubsystem swerveSubsystem, Intake intakeSubsystem, Elevator elevatorSubsystem, Grabber grabberSubsystem) { //add more when more subsystems are made
        //NamedCommands.registerCommand("intakeOn", intakeSubsystem.autoIntake());
        NamedCommands.registerCommand("armUp", grabberSubsystem.setPivotPositionCommand(GrabberPosition.UP));
        NamedCommands.registerCommand("armDown", grabberSubsystem.setPivotPositionCommand(GrabberPosition.DOWN));
        NamedCommands.registerCommand("armOut", grabberSubsystem.setPivotPositionCommand(GrabberPosition.OUT));

        NamedCommands.registerCommand("dropCoral", grabberSubsystem.runGrabberCommand(1).withTimeout(1));

        NamedCommands.registerCommand("defaultArm", elevatorSubsystem.DEF().andThen(new WaitCommand(0.9)).andThen(grabberSubsystem.setPivotPositionCommand(GrabberPosition.DOWN)));
        NamedCommands.registerCommand("pickup", 
        grabberSubsystem.runGrabberCommand(-0.9
          ).withTimeout(2)
          .alongWith(elevatorSubsystem.PICK())
        );


        elevatorCommands("L1", elevatorSubsystem, grabberSubsystem);
        elevatorCommands("L2", elevatorSubsystem, grabberSubsystem);
        elevatorCommands("L3", elevatorSubsystem, grabberSubsystem);
        elevatorCommands("L4", elevatorSubsystem, grabberSubsystem);
        elevatorCommands("LO", elevatorSubsystem, grabberSubsystem);
        elevatorCommands("HI", elevatorSubsystem, grabberSubsystem);



    }

    
    private void elevatorCommands(String name, Elevator elevator, Grabber grabber) {
     NamedCommands.registerCommand(name, elevator.setLevelCommand(ElevatorLevel.valueOf(name))
        .andThen(new WaitCommand(0.3))
        .andThen(grabber.setPivotPositionCommand(GrabberPosition.UP))
     );
    }
}