package frc.robot.subsystems;

import static frc.robot.constants.Constants.LEDConstants.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//https://usermanual.wiki/Pdf/REV20Blinkin20LED20Driver20Users20Manual.1683507547/html#pf8 has all of the PWM color IDs

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWMController;

  public enum LEDState{
    OFF, //black
    DEFAULT, //pink
    CORAL, //white
    ALGAE, //teal
    CLIMB, //blue 
    ISSUE, //strobe red
    EMAIL, // purple 
    AUTO, //pink and black strobe 
    ONREEF, //flashing light blue
    
  }

  //used for controlling lights
  public static LEDState ledState = LEDState.OFF;

  public LEDSubsystem() {
    ledPWMController = new Spark(LED_PWM);
    //sets the PWM port it's wired to on the rio
  }

  public void setLEDMode(LEDMode ledMode) {
    // Sets a LED mode
    ledPWMController.set(ledMode.pwmSignal);
  }
  
  public void setLEDPWM(double PWM) {
    //Sets the PWM signal manually
    ledPWMController.set(PWM);
  }


  // Declare the preset LED modes we want to use (exists to make changing colors easier)
  public enum LEDMode {
    PINK(0.57), //solid hot pink
    GREEN(0.77), //solid green
    WHITE(0.93), // white 
    BLUE(0.87), //solid blue
    STROBE(-0.11), //strobe red'
    PURPLE(0.1),
    OFF(.99); //black

    //red: 0.61 green: 0.77 blue: 0.87

    public double pwmSignal;
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}
