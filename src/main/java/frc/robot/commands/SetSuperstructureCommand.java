package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class SetSuperstructureCommand extends Command {
	private final Elevator elevator;
	private final Pivot pivot;
	private final Supplier<Rotation2d> angleSupplier;
	private final Supplier<Double> heightSupplier;

	public SetSuperstructureCommand(Pivot pivotSubsystem, Elevator elevatorSubsystem,
			Supplier<Rotation2d> desiredAngle, Supplier<Double> desiredHeight) {
		elevator = elevatorSubsystem;
		pivot = pivotSubsystem;
		angleSupplier = desiredAngle;
		heightSupplier = desiredHeight;
		addRequirements(pivot, elevator);
	}

	@Override
	public void initialize() {
		
	}

	@Override
	public void execute() {
		//Rotation2d currentAngle = pivot.getPivotAngle();
		Rotation2d desiredAngle = angleSupplier.get();
		//double currentHeight = elevator.getHeight().in(Meters);
		double desiredHeight = heightSupplier.get();
				
		elevator.setHeight(desiredHeight);
		pivot.setPivotAngle(desiredAngle);
	}

	@Override
	public boolean isFinished() {
		return pivot.atPivotTarget() && elevator.atTarget();
	}
}