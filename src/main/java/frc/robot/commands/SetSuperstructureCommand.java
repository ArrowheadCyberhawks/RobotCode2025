package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import static frc.robot.Constants.GrabberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class SetSuperstructureCommand extends Command {
	private final Grabber grabber;
	private final Elevator elevator;
	private final Supplier<Rotation2d> angleSupplier;
	private final Supplier<Double> heightSupplier;

	public SetSuperstructureCommand(Grabber grabberSubsystem, Elevator elevatorSubsystem,
			Supplier<Rotation2d> desiredAngle, Supplier<Double> desiredHeight) {
		grabber = grabberSubsystem;
		elevator = elevatorSubsystem;
		angleSupplier = desiredAngle;
		heightSupplier = desiredHeight;
		addRequirements(grabber, elevator);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		Rotation2d currentAngle = grabber.getPivotAngle();
		Rotation2d desiredAngle = angleSupplier.get();
		double currentHeight = elevator.getHeight().in(Meters);
		double desiredHeight = heightSupplier.get();

		Rotation2d angleLimit = Rotation2d.fromRadians(Math.acos(currentHeight / kArmLength.in(Meters)));

		// if (!MathUtil.isNear(desiredAngle.getDegrees(), currentAngle.getDegrees(), 3, 0, 360)) { // check if we need to move the arm
		// 	if (currentHeight < ElevatorLevel.CLEAR.getHeight() - 0.01) { // if we are too low to go under
		// 		desiredHeight = ElevatorLevel.CLEAR.getHeight();
		// 		desiredAngle = currentAngle;
		// 		if (desiredHeight < currentHeight) { //if we are too low, but arm still needs to rotate under and elevator still wants to move down
		// 		}
		// 	} else { // if it is safe to go under

		// 	}
		// }

		// if ((currentAngle.getDegrees() > 85 && currentAngle.getDegrees() < 300)
		// 		&& currentHeight < ElevatorLevel.CLEAR.getHeight() - 0.01) { // I HATE this formatting -nitin
		// 	elevator.setHeight(ElevatorLevel.L4.getHeight());
		// 	grabber.setPivotAngle(currentAngle);
		// }
		elevator.setHeight(desiredHeight);
		grabber.setPivotAngle(desiredAngle);
	}

	@Override
	public boolean isFinished() {
		return grabber.atPivotTarget() && elevator.atTarget();
	}
}