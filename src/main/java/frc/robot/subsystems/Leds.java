package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  public static final int LED_LENGTH = 100;
  public int rainbowSpeed = 10;

  AddressableLED strip = new AddressableLED(7);
  AddressableLEDBuffer buf = new AddressableLEDBuffer(LED_LENGTH);
  AddressableLEDSim sim = new AddressableLEDSim(strip);
  int rainbowCounter = 0;

  public Leds() {
    strip.setLength(LED_LENGTH);
    strip.start();
    setDefaultCommand(runPattern(rainbow(255, 128)));
  }

  @Override
  public void periodic() {
    strip.setData(buf);
  }

  public Command runPattern(LEDPattern pattern) {
    return run(()-> {pattern.applyTo(buf);}).ignoringDisable(true);
  }


  /* WPILib LEDPattern.rainbow except we're shifting the hue of the first led */
  LEDPattern rainbow(int saturation, int value) {
    return (reader, writer) -> {
      int firstLedHue = rainbowCounter;
      int bufLen = reader.getLength();
      for (int i = 0; i < bufLen; i++) {
        int hue = ((i * 180) / bufLen + firstLedHue) % 180;
        writer.setHSV(i, hue, saturation, value);
      }
      rainbowCounter += rainbowSpeed;
      rainbowCounter %= 180;
    };
  }
}
