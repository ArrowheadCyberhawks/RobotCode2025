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
  public static LEDState ledState = LEDState.DEF;

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
    RAINBOW(-0.99), // fixed palette pattern, rainbow
    PARTY(-0.97), // fixed palette pattern, party
    OCEAN(-0.95), // fixed palette pattern, ocean
    LAVENDER(-0.93), // fixed palette pattern, lavender
    FOREST(-0.91), // fixed palette pattern, forest
    GLITTER(-0.89), // fixed palette pattern, rainbow with glitter
    CONFETTI(-0.87), // fixed palette pattern, confetti
    REDSHOT(-0.85), // fixed palette pattern, shot, red
    BLUESHOT(-0.83), // fixed palette pattern, shot, blue
    WHITESHOT(-0.81), // fixed palette pattern, shot, white
    SINELONRAINBOW(-0.79), // fixed palette pattern, sinelon, rainbow
    SINELONPARTY(-0.77), // fixed palette pattern, sinelon, party
    SINELONOCEAN(-0.75), // fixed palette pattern, sinelon, ocean
    SINELONLAVENDER(-0.73), // fixed palette pattern, sinelon, lava
    SINELONFOREST(-0.71), // fixed palette pattern, sinelon, forest
    BEATSRGB(-0.69), // fixed palette pattern, beats per minute, rainbow
    BEATSPARTY(-0.67), // fixed palette pattern, beats per minute, party
    BEATSOCEAN(-0.65), // fixed palette pattern, beats per minute, ocean
    BEATSLAVENDER(-0.63), // fixed palette pattern, beats per minute, lava
    BEATSFOREST(-0.61), // fixed palette pattern, beats per minute, forest
    FIREMEDIUM(-0.59), // fixed palette pattern, fire, medium
    FIRELARGE(-0.57), // fixed palette pattern, fire, large
    TWINKLESRAINBOW(-0.55), // fixed palette pattern, twinkles, rainbow
    TWINKLESPARTY(-0.53), // fixed palette pattern, twinkles, party
    TWINKLESOCEAN(-0.51), // fixed palette pattern, twinkles, ocean
    TWINKLESLAVENDER(-0.49), // fixed palette pattern, twinkles, lava
    TWINKLESFOREST(-0.47), // fixed palette pattern, twinkles, forest
    COLORWAVESRAINBOW(-0.45), // fixed palette pattern, color waves, rainbow
    COLORWAVESPARTY(-0.43), // fixed palette pattern, color waves, party
    COLORWAVESOCEAN(-0.41), // fixed palette pattern, color waves, ocean
    COLORWAVESLAVENDER(-0.39), // fixed palette pattern, color waves, lava
    COLORWAVESFOREST(-0.37), // fixed palette pattern, color waves, forest
    LARSONSCANNERRED(-0.35), // fixed palette pattern, larson scanner, red
    LARSONSCANNERGRAY(-0.33), // fixed palette pattern, larson scanner, gray
    LIGHTCHASERED(-0.31), // fixed palette pattern, light chase, red
    LIGHTCHASEBLUE(-0.29), // fixed palette pattern, light chase, blue
    LIGHTCHASEGRAY(-0.27), // fixed palette pattern, light chase, gray
    HEARTBEATRED(-0.25), // fixed palette pattern, heartbeat, red
    HEARTBEATBLUE(-0.23), // fixed palette pattern, heartbeat, blue
    HEARTBEATWHITE(-0.21), // fixed palette pattern, heartbeat, white
    HEARTBEATGRAY(-0.19), // fixed palette pattern, heartbeat, gray
    BREATHRED(-0.17), // fixed palette pattern, breath, red
    BREATHBLUE(-0.15), // fixed palette pattern, breath, blue
    BREATHGRAY(-0.13), // fixed palette pattern, breath, gray
    STROBERED(-0.11), // fixed palette pattern, strobe, red
    STROBEBLUE(-0.09), // fixed palette pattern, strobe, blue
    STROBEGOLD(-0.07), // fixed palette pattern, strobe, gold
    STROBEWHITE(-0.05), // fixed palette pattern, strobe, white
    END2ENDBLACK(-0.03), // color 1, pattern end to end blend to black
    LARSONSCANNER1(0.01), // color 1, pattern larson scanner
    LIGHTCHASEDIM(0.03), // color 1, pattern light chase dimming
    HEARTBEATSLOW(0.05), // color 1, pattern heartbeat slow
    HEARTBEATMEDIUM(0.07), // color 1, pattern heartbeat medium
    HEARTBEATFAST(0.09), // color 1, pattern heartbeat fast
    BREATHSLOW(0.11), // color 1, pattern breath slow
    BREATHFAST(0.13), // color 1, pattern breath fast
    SHOT(0.15), // color 1, pattern shot
    STROBE(0.17), // color 1, pattern strobe
    END2ENDBLEND2BLACK(0.19), // color 2, pattern end to end blend to black
    LARSONSCANNER2(0.21), // color 2, pattern larson scanner
    LIGHTCHASEDIM2(0.23), // color 2, pattern light chase dimming
    HEARTBEATSLOW2(0.25), // color 2, pattern heartbeat slow
    HEARTBEATMEDIUM2(0.27), // color 2, pattern heartbeat medium
    HEARTBEATFAST2(0.29), // color 2, pattern heartbeat fast
    BREATHSLOW2(0.31), // color 2, pattern breath slow
    BREATHFAST2(0.33), // color 2, pattern breath fast
    SHOT2(0.35), // color 2, pattern shot
    STROBE2(0.37), // color 2, pattern strobe
    SPARKLECOLOR1(0.39), // color 1 and 2, pattern sparkle, color 1 on color 2
    SPARKLECOLOR2(0.41), // color 1 and 2, pattern sparkle, color 2 on color 1
    COLORGRADIENT(0.43), // color 1 and 2, pattern color gradient, color 1 and 2
    BEATSPERMINUTECOLOR1AND2(0.45), // color 1 and 2, pattern beats per minute, color 1 and 2
    END2ENDBLENDCOLOR1TO2(0.47), // color 1 and 2, pattern end to end blend, color 1 to 2
    END2ENDBLEND(0.49), // color 1 and 2, pattern end to end blend
    NOBLENDING(0.51), // color 1 and 2, pattern color 1 and color 2 no blending
    TWINKLESCOLOR1AND2(0.53), // color 1 and 2, pattern twinkles, color 1 and 2
    COLORWAVESCOLOR1AND2(0.55), // color 1 and 2, pattern color waves, color 1 and 2
    SINELONCOLOR1AND2(0.57), // color 1 and 2, pattern sinelon, color 1 and 2
    HOTPINK(0.57), // solid colors, hot pink
    DARKRED(0.61), // solid colors, dark red
    RED(0.61), // solid colors, red
    REDORANGE(0.63), // solid colors, red orange
    ORANGE(0.65), // solid colors, orange
    GOLD(0.67), // solid colors, gold
    YELLOW(0.69), // solid colors, yellow
    LAWNGREEN(0.71), // solid colors, lawn green
    LIME(0.73), // solid colors, lime
    DARKGREEN(0.75), // solid colors, dark green
    GREEN(0.77), // solid colors, green
    BLUEGREEN(0.79), // solid colors, blue green
    AQUA(0.81), // solid colors, aqua
    SKYBLUE(0.83), // solid colors, sky blue
    DARKBLUE(0.85), // solid colors, dark blue
    BLUE(0.87), // solid colors, blue
    BLUEVIOLET(0.89), // solid colors, blue violet
    VIOLET(0.91), // solid colors, violet
    WHITE(0.93), // solid colors, white
    GRAY(0.95), // solid colors, gray
    DARKGRAY(0.97), // solid colors, dark gray
    BLACK(0.99); // solid colors, black
    //red: 0.61 green: 0.77 blue: 0.87

    public double pwmSignal;
    LEDMode(double pwmSignal) {
      this.pwmSignal = pwmSignal;
    }
  }
}
