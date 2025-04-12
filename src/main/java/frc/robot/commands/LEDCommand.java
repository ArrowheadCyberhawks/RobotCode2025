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
/* 
    public void execute(){
        setLEDMode(LEDState.OFF, LEDMode.OFF);
        setLEDMode(LEDState.ALGAE, LEDMode.GREEN);
        setLEDMode(LEDState.CORAL, LEDMode.WHITE);
        setLEDMode(LEDState.CLIMB, LEDMode.BLUE);
        setLEDMode(LEDState.EMAIL, LEDMode.PURPLE);
        setLEDMode(LEDState.DEFAULT, LEDMode.PINK);
        setLEDMode(LEDState.ISSUE, LEDMode.STROBE);



    }
*/

    public void execute(){
        setLEDMode(LEDState.OFF, LEDMode.OFF);
        setLEDMode(LEDState.ALGAE, LEDMode.GREEN);
        setLEDMode(LEDState.CORAL, LEDMode.WHITE);
        setLEDMode(LEDState.CLIMB, LEDMode.BLUE);
        setLEDMode(LEDState.EMAIL, LEDMode.PURPLE);
        setLEDMode(LEDState.DEFAULT, LEDMode.PINK);
        setLEDMode(LEDState.ISSUE, LEDMode.STROBE);



    }
    public boolean isFinished(){
        return false;
    }
        
    public void periodic() {
        //might have to change to a different color idrk
        ledSubsystem.setLEDPWM(0.93);
    }

    /**
     * If the LEDs are set to a certain state, it will change the color to the corresponding color
     * @param state The LED State of the robot
     * @param mode The desired LED Mode/Color
     */
    public void setLEDMode(LEDState state, LEDMode mode) {
        if(LEDSubsystem.ledState == state){
            ledSubsystem.setLEDMode(mode);
        }
    }
    
    }
    