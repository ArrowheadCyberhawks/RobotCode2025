package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Grabber;

public class SetGrabberPositionCommand extends Command {
  private final Grabber grabber;
  private final Elevator elevator;
  private final MutAngle desiredAngle;

  public SetGrabberPositionCommand (Grabber grabberSubsystem, Elevator elevatorSubsystem, MutAngle desiredAngle) {
    grabber = grabberSubsystem;
    elevator = elevatorSubsystem;
    this.desiredAngle = desiredAngle;
    addRequirements(grabber, elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Angle currentAngle = grabber.getPivotAngle().getMeasure();

    // check if we're going to hit the bottom of the robot
    if (desiredAngle.minus(currentAngle).abs(Degrees) > 180 || (desiredAngle.in(Degrees) > 90 && currentAngle.in(Degrees) < 270)) {
      elevator.setLevelCommand(ElevatorLevel.CLEAR);
    }
    grabber.setPivotAngle(new Rotation2d(desiredAngle));
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}