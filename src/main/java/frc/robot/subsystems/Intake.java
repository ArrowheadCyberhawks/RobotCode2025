// package frc.robot.subsystems;

// // import static frc.robot.Constants.IntakeConstants.*;

// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /**
//  * Subsystem for the intake. Includes a motor to intake game pieces and a motor to extend the intake.
//  */
// public class Intake extends SubsystemBase {
//     private final SparkMax intakeMotor, extendMotor;
//     private final SparkClosedLoopController extendController;
//     /**
//      * Enum to represent the extension state of the intake mechanism.
//      */
//     public enum ExtendState {
//         EXTENDED,
//         RETRACTED
//     }
    
//     /**
//      * Creates a new Intake subsystem using the motor ports defined in Constants.
//      */
//     public Intake() {
//         intakeMotor = new SparkMax(kIntakeMotorPort, SparkMax.MotorType.kBrushless);
//         extendMotor = new SparkMax(kExtendMotorPort, SparkMax.MotorType.kBrushless);
//         extendController = extendMotor.getClosedLoopController();
//     }


//     /**
//      * Stops the intake motor.
//      */
//     public void stopIntakeMotor() {
//         intakeMotor.stopMotor();
//     }

//     public void stopExtendMotor() {
//         extendMotor.stopMotor();
//     }
//     public void setIntakeMotor(double speed) {
//         intakeMotor.set(speed);
//     }

//     public void setExtendMotor(double speed){
//         extendMotor.set(speed);
//     }

//     /**
//      * Sets the state of the intake extension.
//      * 
//      * @param state The state to set the intake extension to, either extended or retracted.
//      */
//     public void setState(ExtendState state) {
//         if (state == ExtendState.EXTENDED) {
//             extendController.setReference(1, ControlType.kMAXMotionPositionControl);
//         } else {
//             extendController.setReference(0, ControlType.kMAXMotionPositionControl);
//         }
//     }

//     /**
//      * Runs the intake motor at a given speed, then stops it when the command ends.
//      * 
//      * @param speed The percent speed to run the intake motor at. Should be between -1 and 1.
//      * @return A command that runs the intake motor at a given speed.
//      */
//     public Command runIntakeCommand(double speed) {
//         return this.runEnd(() -> setIntakeMotor(speed), () -> stopIntakeMotor());
//     }

//     public Command runRetractCommand(double speed) {
//         return this.runEnd(() -> setExtendMotor(speed), () -> stopExtendMotor());
//     }

//     public Command runExtendCommand(double speed) {
//         return this.runEnd(() -> setExtendMotor(-speed), () -> stopExtendMotor());
//     }


//     /**s
//      * Extends the intake.
//      * 
//      * @return A command that extends the intake.
//      */
//     public Command extendCommand() {
//         return this.runOnce(() -> setState(ExtendState.EXTENDED));
//     }

//     /**
//      * Retracts the intake.
//      * 
//      * @return A command that retracts the intake.s
//      */
//     public Command retractCommand() {
//         return this.runOnce(() -> setState(ExtendState.RETRACTED));
//     }

// }
