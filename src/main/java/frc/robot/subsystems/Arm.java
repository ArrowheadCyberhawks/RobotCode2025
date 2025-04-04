package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.Constants.GrabberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.GrabberConstants.PivotPosition;

/**
 * The Grabber subsystem covers the motors that manipulate the game piece
 * as well as the pivoting mechanism.
 */
public class Arm extends SubsystemBase {
    private final SparkMax pivotMotor = new SparkMax(kPivotMotorPort, MotorType.kBrushless);
    private final ProfiledPIDController pivotController = new ProfiledPIDController(kPivotP.get(), kPivotI.get(), kPivotD.get(), new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get())); //0.09    
    private final CANcoder pivotEncoder = new CANcoder(kPivotEncoderId); //0.09
    private final StatusSignal<Angle> pivotEncoderAngle = pivotEncoder.getAbsolutePosition();
    private final ArmFeedforward pivotFeedforward = new ArmFeedforward(kPivotS.get(), kPivotG.get(), kPivotV.get(), kPivotA.get()); //0.09

    /**
     * Creates a new Grabber subsystem using the motor ports defined in Constants.
     */
    public Arm() {
        pivotController.setGoal(getPivotAngle().getRadians());
        pivotController.setTolerance(Units.degreesToRadians(3));
        // pivotController.enableContinuousInput(0, 2 * Math.PI);
    }


    public void periodic() {
        updateConstants();
        // if (Math.abs(getPivotAngle().getDegrees()) < 15) {
        //     pivotController.setP(kPivotUpP.get());
        // } else {
        //     pivotController.setP(kPivotP.get());
        // }
        // setPivotMotor(MathUtil.clamp(pivotController.calculate(getPivotAngle().getRadians()), -kMaxPivotPower, kMaxPivotPower));
        
        pivotMotor.setVoltage(Volts.of(pivotFeedforward.calculate(pivotController.getSetpoint().position, pivotController.getSetpoint().velocity) + pivotController.calculate(getPivotAngle().getRadians())));

        SmartDashboard.putNumber("Pivot Angle", getPivotAngle().getRadians());
        SmartDashboard.putNumber("Pivot Target", pivotController.getGoal().position);
        // logging
        Logger.recordOutput(getName() + "/Pivot Angle", getPivotAngle().getRadians());
        Logger.recordOutput(getName() + "/Pivot Target", pivotController.getGoal().position);
    }

    private void updateConstants() {
        if (kPivotP.get() != pivotController.getP()
                || kPivotI.get() != pivotController.getI()
                || kPivotD.get() != pivotController.getD()
                || kPivotS.get() != pivotFeedforward.getKs()
                || kPivotG.get() != pivotFeedforward.getKg()
                || kPivotV.get() != pivotFeedforward.getKv()
                || kPivotA.get() != pivotFeedforward.getKa()
                || kPivotMaxAccel.get() != pivotController.getConstraints().maxAcceleration
                || kPivotMaxVel.get() != pivotController.getConstraints().maxVelocity) {
            pivotController.setP(kPivotP.get());
            pivotController.setI(kPivotI.get());
            pivotController.setD(kPivotD.get());
            pivotFeedforward.setKs(kPivotS.get());
            pivotFeedforward.setKg(kPivotG.get());
            pivotFeedforward.setKv(kPivotV.get());
            pivotFeedforward.setKa(kPivotA.get());
            pivotController.setConstraints(new Constraints(kPivotMaxVel.get(), kPivotMaxAccel.get()));
        }
    }

    /**
     * Sets the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is up, positive is clockwise looking at the front of the r.
     */
    public void setPivotAngle(Rotation2d angle) {
        if (Math.abs(angle.getDegrees()) < kPivotLimit.getDegrees()) {
            return;
        }
        pivotController.setGoal(angle.getRadians());
    }
    
    public void setPivotPosition(PivotPosition position) {
        setPivotAngle(position.getAngle());
    }

    public void resetPivotTarget() {
        pivotController.setGoal(getPivotAngle().getRadians());
    }

    /**
     * Sets the speed of the pivot motor.
     * @param speed The percent speed to set the motor to. Should be between -1 and 1.
     */
    public void setPivotMotor(double speed) {
        pivotMotor.set(speed);
    }
    
    /**
     * Returns the position of the pivot motor.
     * @return A Rotation2d representing the position of the pivot motor encoder.
     */
    public Rotation2d getPivotAngle() {
        pivotEncoderAngle.refresh();
        return Rotation2d.fromRotations(pivotEncoderAngle.getValueAsDouble());
    }

    public void stopPivotMotor() {
        pivotMotor.stopMotor();
    }

    public boolean atPivotTarget() {
        return pivotController.atGoal();
    }

    public void resetPivotAngle(Rotation2d angle) {
        pivotEncoder.setPosition(angle.getRotations());
    }

    /**
     * Command to set the pivot angle of the grabber.
     * @param angle The Rotation2d to set the pivot to. 0 is horizontal, positive is up.
     * @return A command that sets the pivot angle of the grabber.
     */
    public Command setPivotAngleCommand(Rotation2d angle) {
        return runOnce(() -> setPivotAngle(angle));
    }

    public Command setPivotPositionCommand(PivotPosition position) {
        return runOnce(() -> setPivotPosition(position));
    }

    public Command runPivotCommand(double speed) {
        return runEnd(() -> setPivotMotor(speed), () -> stopPivotMotor());
    }
}