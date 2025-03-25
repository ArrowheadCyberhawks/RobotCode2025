package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.LEDSubsystem.LEDState;


public class LEDCommand extends Command{

    private final LEDSubsystem ledSubsystem;

    public LEDCommand(LEDSubsystem ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    public void initialize(){
    }

    public void execute(){
        setLEDMode(LEDState.OFF, LEDMode.OFF);
        setLEDMode(LEDState.IN, LEDMode.IN);
        setLEDMode(LEDState.OUT, LEDMode.OUT);

        // if(LEDSubsystem.ledState == LEDState.OFF){
        //     ledSubsystem.setLEDMode(LEDMode.OFF);
        // }
        // if(LEDSubsystem.ledState == LEDState.INTAKE){
        //     ledSubsystem.setLEDMode(LEDMode.INTAKE);
        // }
        //  if(LEDSubsystem.ledState == LEDState.RING){
        //     ledSubsystem.setLEDMode(LEDMode.RING);
        // }
        //  if(LEDSubsystem.ledState == LEDState.SHOOT){
        //     ledSubsystem.setLEDMode(LEDMode.SHOOT);
        // }
    }    

    public boolean isFinished(){
        return false;
    }

    public void periodic() {
        ledSubsystem.setLEDPWM(0.93);
    }

    public void setLEDMode(LEDState state, LEDMode mode) {
        if(LEDSubsystem.ledState == state){
            ledSubsystem.setLEDMode(mode);
        }
    }
    
}