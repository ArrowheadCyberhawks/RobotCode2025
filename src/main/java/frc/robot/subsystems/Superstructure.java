package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;



public class Superstructure extends SubsystemBase{

    //CHANGE THIS ENTIRE THING SO IT'S A COMMAND
    //IT BASICALLY FUNCTIONS LIKE ONE
    //IT CAN'T BE THAT HARD
     
    public static Elevator elevator;
    public static Pivot pivot;

    public static enum SuperStructureState {
        DEF,
        LO,
        L1,
        L2,
        L3,
        L4,
        ALG2,
        ALG3,
        BARGE,
        PROCESSOR,
        PICKUP,
        INTAKE
    }

    public static SuperStructureState superStructureState = SuperStructureState.LO;
    public static SuperStructureState nextSuperStructureState = SuperStructureState.L3;

    public Superstructure(Elevator elevator, Pivot pivot) {
        Superstructure.elevator = elevator;
        Superstructure.pivot = pivot;
    }

    //NOTE: This ONLY WORKS if manual controls don't put it in a dangerous position


    public Command Intake() {
        superStructureState = SuperStructureState.INTAKE;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.DOWN::getAngle, ElevatorLevel.LO::getHeight);
        if (superStructureState == SuperStructureState.L1 ||
                superStructureState == SuperStructureState.L2 ||
                superStructureState == SuperStructureState.LO ||
                superStructureState == SuperStructureState.PROCESSOR) {
            return Clear(end);
        } else {
            return end;
        }
    }

    public Command LO() {
        superStructureState = SuperStructureState.LO;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.ZERO::getAngle, ElevatorLevel.LO::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    public Command PICKUP() {
        superStructureState = SuperStructureState.LO;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.ZERO::getAngle, ElevatorLevel.LO::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }


    public Command Processor() {
        superStructureState = SuperStructureState.PROCESSOR;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.PROC::getAngle, ElevatorLevel.LO::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    public Command L1() {
        superStructureState = SuperStructureState.L1;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.L1::getAngle, ElevatorLevel.L1::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    public Command L2() {
        superStructureState = SuperStructureState.L2;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.PLACE::getAngle, ElevatorLevel.L2::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    //double check to see if L3 hits...
    public Command L3() {
        superStructureState = SuperStructureState.L3;
        return new SetSuperstructureCommand(pivot, elevator, PivotPosition.PLACE::getAngle, ElevatorLevel.L3::getHeight);
    }

    public Command L4() {
        superStructureState = SuperStructureState.L4;
        return new SetSuperstructureCommand(pivot, elevator, PivotPosition.L4::getAngle, ElevatorLevel.L4::getHeight);
    }

    public Command Barge() {
        superStructureState = SuperStructureState.BARGE;
        return new SetSuperstructureCommand(pivot, elevator, PivotPosition.HI::getAngle, ElevatorLevel.HI::getHeight);
    }

    public Command Algae2() {
        superStructureState = SuperStructureState.ALG2;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.ALGREEF::getAngle, ElevatorLevel.ALG2::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    public Command Algae3() {
        superStructureState = SuperStructureState.ALG3;
        return new SetSuperstructureCommand(pivot, elevator, PivotPosition.ALGREEF::getAngle, ElevatorLevel.ALG3::getHeight);
    }


    public Command Clear(Command c) {
        return new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
        .withTimeout(0.5)
        .andThen(c);
    }

    public Command getNextSuperStructure(SuperStructureState state) {

        //tad bit monkey but states
        switch (state) {
            case ALG3:
                return Algae3();
            case ALG2:
                return Algae2();
            case BARGE:
                return Barge();
            case L4:
                return L4();
            case L3:
                return L3();
            case L2:
                return L2();
            case L1:
                return L1();
            case PROCESSOR:
                return Processor();
            case INTAKE:
                return Intake();
            case PICKUP:
                return PICKUP();
            default:
                return LO();
        }
    }

    public Command setNextSuperStructure(SuperStructureState state) {
        setSuperstructureState(state);
        return runOnce(() -> nextSuperStructureState = state)
            .andThen(Commands.print(nextSuperStructureState.toString() + " " + state.toString()));
    }

    public void setSuperstructureState(SuperStructureState state) {
        nextSuperStructureState = state;
    }

}    


