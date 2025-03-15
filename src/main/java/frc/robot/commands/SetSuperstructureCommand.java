package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import static frc.robot.Constants.GrabberConstants.*;
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

  public SetSuperstructureCommand (Grabber grabberSubsystem, Elevator elevatorSubsystem, Supplier<Rotation2d> desiredAngle, Supplier<Double> desiredHeight) {
    grabber = grabberSubsystem;
    elevator = elevatorSubsystem;
    angleSupplier = desiredAngle;
    heightSupplier = desiredHeight;
    addRequirements(grabber, elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Rotation2d currentAngle = grabber.getPivotAngle();
    Rotation2d desiredAngle = angleSupplier.get();
    double currentHeight = elevator.getHeight().in(Meters);
    double desiredHeight = heightSupplier.get();

    Rotation2d angleLimit = Rotation2d.fromRadians(Math.acos(currentHeight / kArmLength.in(Meters)));

    // check if we're going to hit the bottom of the robot
    if (
        (Math.abs(desiredAngle.minus(currentAngle).getDegrees()) > 180
        || (currentAngle.getDegrees() > 85 && currentAngle.getDegrees() < 275))
      && currentHeight < ElevatorLevel.CLEAR.getHeight() - 0.01) { //I HATE this formatting -nitin
      elevator.setHeight(ElevatorLevel.CLEAR.getHeight());
      grabber.setPivotAngle(currentAngle);
    } else {
      elevator.setHeight(desiredHeight);
      grabber.setPivotAngle(desiredAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return grabber.atPivotTarget() && elevator.atTarget();
  }
}