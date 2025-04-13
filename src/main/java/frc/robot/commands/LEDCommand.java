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
        checkState(LEDState.OFF, LEDMode.DARKGRAY);
        checkState(LEDState.CORAL, LEDMode.WHITE);
        checkState(LEDState.ALGAE, LEDMode.GREEN);
        checkState(LEDState.EMAIL, LEDMode.ORANGE);
        checkState(LEDState.CLIMB, LEDMode.BLUE);
        checkState(LEDState.ONREEF, LEDMode.GREEN);
        checkState(LEDState.ISSUE, LEDMode.RED);
        checkState(LEDState.DEFAULT, LEDMode.HOTPINK);
        checkState(LEDState.AUTO, LEDMode.VIOLET);
        checkState(LEDState.ISCLIMBED, LEDMode.GLITTER);


    }    

    public boolean isFinished(){
        return false;
    }

    public void periodic() {
        //might have to change to a different color idrk
        //LEDSubsystem.ledState = LEDState.DEF; //BROKEN CODE NITIN U BUM
    }

    /**
     * If the LEDs are set to a certain state, it will change the color to the corresponding color
     * @param state The LED State of the robot
     * @param mode The desired LED Mode/Color
     */
    public void checkState(LEDState state, LEDMode mode) {
        if(LEDSubsystem.ledState == state){
            ledSubsystem.setLEDMode(mode);
        }
    }
    
}