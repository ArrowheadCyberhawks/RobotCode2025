package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.*;
import frc.robot.constants.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.constants.Constants.GrabberConstants.GrabberState;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;
import static edu.wpi.first.units.Units.*;



public class Superstructure extends SubsystemBase{

    //CHANGE THIS ENTIRE THING SO IT'S A COMMAND
    //IT BASICALLY FUNCTIONS LIKE ONE
    //IT CAN'T BE THAT HARD
     
    public static Elevator elevator;
    public static Arm pivot;
    public static Grabber grabber;

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

    public static SuperStructureState superStructureState = SuperStructureState.L3;
    public static SuperStructureState nextSuperStructureState = SuperStructureState.L3;

    public Superstructure(Elevator elevator, Arm pivot, Grabber grabber) {
        Superstructure.elevator = elevator;
        Superstructure.pivot = pivot;
        Superstructure.grabber = grabber;
        
    }

    //NOTE: This ONLY WORKS if manual controls don't put it in a dangerous position, so just have height checks for each one seperately
    //and maybe demonkey it at some point this is horrid


    public Command Intake() {
        superStructureState = SuperStructureState.INTAKE;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.HUMAN::getAngle, ElevatorLevel.HUMAN::getHeight);
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
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.LO::getAngle, ElevatorLevel.LO::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    public Command Pickup() {
        superStructureState = SuperStructureState.LO;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.LO::getAngle, ElevatorLevel.LO::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }


    public Command Processor() {
        superStructureState = SuperStructureState.PROCESSOR;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.ALGPICK::getAngle, ElevatorLevel.LO::getHeight);
        return superStructureState == SuperStructureState.INTAKE ? Clear(end) : end;
    }

    public Command L1() {
        superStructureState = SuperStructureState.L1;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.LO::getAngle, ElevatorLevel.L1::getHeight);
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
        // Note: This can be simplified down to bringing the pivot to a position and then bringing the elevator up, shooting once past a certain height
        // Only use that method if time constraints deem it hard to do, or if the arm slop results in unexpected effects.
        return new SequentialCommandGroup(
            pivot.setPivotAngleCommand(PivotPosition.LO.getAngle()), //switch to a low position
            new ParallelCommandGroup(
                elevator.setLevelCommand(ElevatorLevel.HI), //brings the elevator up
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> elevator.getHeight().in(Meters) < 1.3), //waits until the elevator is high enough
                    new ParallelCommandGroup(
                        pivot.setPivotAngleCommand(PivotPosition.HI.getAngle()), //brings the pivot out
                        new SequentialCommandGroup(
                            new WaitUntilCommand(() -> pivot.getPivotAngle().getRadians() < Math.PI/2), //TODO change number
                            grabber.outtakeCommand() //shoots once past shooting angle
                        )
                    )
                )
            )
        );
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

    public Command AlgaePickup() {
        superStructureState = SuperStructureState.PICKUP;
        Command end = new SetSuperstructureCommand(pivot, elevator, PivotPosition.ALGPICK::getAngle, ElevatorLevel.LO::getHeight);
        return end;//elevator.getHeight().in(Meters) < ElevatorLevel.CLEAR.getHeight() ? Clear(end) : end;
    }

    public Command Clear(Command c) {
        return new SetSuperstructureCommand(pivot, elevator, pivot::getPivotAngle, ElevatorLevel.CLEAR::getHeight)
        .withTimeout(0.5)
        .andThen(c);
    }

    
	/**
	 * Automatically throws the algae onto the barge and brings the superstructure back down.
	 * @return The command.
	 */
	public Command bargePlace() {
		return new ParallelCommandGroup(
			elevator.setLevelCommand(ElevatorLevel.HI),
			pivot.setPivotPositionCommand(PivotPosition.HI),
			grabber.runGrabberCommand(GrabberState.OUTTAKE_A::getSpeed)
				.withTimeout(0.75)
				.beforeStarting(
					new WaitUntilCommand(() -> elevator.getHeight().in(Meters) > ElevatorLevel.HI.getHeight() - 0.25)
				)
		).andThen(LO());
	}

	public Command reefPick() {
		return new SequentialCommandGroup(
				pivot.setPivotPositionCommand(PivotPosition.ALGREEF)
				.alongWith(grabber.runGrabberCommand(GrabberState.INTAKE::getSpeed).withTimeout(2)),
				grabber.intakeCommand().withTimeout(0.1),
				pivot.setPivotPositionCommand(PivotPosition.LO)
			)
			.repeatedly()
			.until(() -> grabber.hasAlgae() && pivot.getPivotAngle().getRadians() > PivotPosition.LO.getAngle().getRadians() - 0.1)
			.andThen(elevator.setLevelCommand(ElevatorLevel.LO));
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
                return Pickup();
            default:
                return LO();
        }
    }

    public void periodic() {

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