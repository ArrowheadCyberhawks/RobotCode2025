package frc.robot.subsystems;

import static frc.robot.constants.Constants.LEDConstants.*;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//https://usermanual.wiki/Pdf/REV20Blinkin20LED20Driver20Users20Manual.1683507547/html#pf8 has all of the PWM color IDs

public class LEDSubsystem extends SubsystemBase {

  private Spark ledPWMController;

  public enum LEDState{
    OFF, //black
    IN, //violet
    OUT, //red
    DEF, //pink
    ALIGN, //blue
    READY, //green
    ERROR, //strobe red
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
    RED(0.61),
    BLUE(0.87), //solid blue
    STROBERED(-0.11), //strobe red
    VIOLET(0.91), //violet
    OFF(.99); //black
    //red: 0.61 green: 0.77 blue: 0.87

    public double pwmSignal;
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}
