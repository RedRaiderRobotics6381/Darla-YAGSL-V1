// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDsSubSystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private int step = 0;
  private boolean isOn = false;
  private int currentLed = 0;
  private boolean direction = true;

  public LEDsSubSystem() {
    m_led = new AddressableLED(1); // Set the LED PWM port to 0

    // Reuse buffer, setting length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(23);  //Set the buffer length to 23
    m_led.setLength(m_ledBuffer.getLength()); //Set the length of the LED buffer
    m_led.start();// Start the LED buffer
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer); // Set the data of the LED buffer
  }

  /**
   * Sets all LEDs to a solid color.
   * 
   * @param hue    the hue value of the color (0-255)
   * @param value  the value/brightness of the color (0-255)
   * @return       the command object
   */
  public Command setSolidLED(int hue, int value) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) { // For every pixel
      m_ledBuffer.setHSV(i, hue, 255, value); // Set the HSV value of the pixel
    }
    return null;
  }
  
  /**
   * Executes a fade effect on the LEDs with the specified hue and delay.
   * 
   * @param hue   the hue value for the LEDs
   * @param delay the delay in milliseconds between each step of the fade effect
   * @return the Command object representing the fade effect
   */
  public Command fadeEffect(int hue, long delay) {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      int val = (int) ((Math.sin(step / 23.0 * 2 * Math.PI) + 1) / 2 * (150 - 30) + 30);
      m_ledBuffer.setHSV(i, hue, 255, val);
    }
    m_led.setData(m_ledBuffer); // Set the data of the LED buffer
    step++; // Increase the step
    if (step >= 23) step = 0; // Reset the step when it reaches 23

    try {
      Thread.sleep(delay); // Delay for 100 milliseconds
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    return null;
  }
  
  /**
   * Represents a command that controls the LEDs' fire effect.
   * This command sets the HSV values of the LED pixels to create a flickering effect resembling a flame.
   * The command never finishes and returns null.
   *
   * @return null
   */
  public Command fireEffect() {
    Random random = new Random();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Generate a random brightness value that resembles the flickering of a flame
            int brightness = random.nextInt(255);

            // Set the hue to a value that resembles the color of a flame
            int hue = random.nextInt(45);

            m_ledBuffer.setHSV(i, hue, 255, brightness); // Set the HSV value of the pixel
        }
        m_led.setData(m_ledBuffer); // Set the data of the LED buffer

        try {
            Thread.sleep(250); // Sleep for a short interval to create a flickering effect
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return null; // This command never finishes
    }

  
  /**
   * This class provides the strobeEffect method which sets the HSV value of each pixel in the LED buffer to create a strobe effect.
   * The method takes in the hue, value, and interval parameters to control the appearance and timing of the effect.
   * 
   * @param hue the hue value for the strobe effect
   * @param value the value (brightness) value for the strobe effect
   * @param interval the interval in milliseconds between each change in the strobe effect
   * @return null
   */
  public Command strobeEffect(int hue, int value, long interval) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          if (isOn) {
              m_ledBuffer.setHSV(i, hue, 255, value); // Set the HSV value of the pixel
          } else {
              m_ledBuffer.setHSV(i, hue, 255, 0); // Turn off the pixel
          }
      }
      m_led.setData(m_ledBuffer); // Set the data of the LED buffer

      try {
          Thread.sleep(interval); // Sleep for the specified interval
      } catch (InterruptedException e) {
          e.printStackTrace();
      }

      isOn = !isOn; // Toggle the state
      return null;
  }

  /**
   * Represents a command that controls the LEDs scan effect.
   * This command continuously scans through the LEDs, turning them on and off in a pattern.
   *
   * @param hue The hue value for the LEDs.
   * @param value The value (brightness) for the LEDs.
   * @param interval The time interval between each LED update.
   * @return null, as this command never finishes.
   */
  public Command scanEffect(int hue, int value, long interval) {
    // Turn off all LEDs
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setHSV(i, hue, 255, (int) (value * 0.25));
    }

    // Turn on the current LED
    m_ledBuffer.setHSV(currentLed, hue, 255, value); // Set the HSV value of the pixel

    int previousLed = currentLed - 1;
    if (previousLed >= 0) {
      m_ledBuffer.setHSV(previousLed, hue, 255, (int) (value * 0.75));
    }

    int prevPreviousLed = currentLed - 2;
    if (prevPreviousLed >= 0) {
      m_ledBuffer.setHSV(prevPreviousLed, hue, 255, (int) (value * 0.50));
    }

    int nextLed = currentLed + 1;
    if (nextLed < m_ledBuffer.getLength()) {
      m_ledBuffer.setHSV(nextLed, hue, 255, (int) (value * 0.75));
    }

    int nextNextLed = currentLed + 2;
    if (nextNextLed < m_ledBuffer.getLength()) {
      m_ledBuffer.setHSV(nextNextLed, hue, 255, (int) (value * 0.50));
    }
    
    m_led.setData(m_ledBuffer); // Set the data of the LED buffer

    // Update the current LED based on the direction
    if (direction) {
      currentLed++;
      if (currentLed >= m_ledBuffer.getLength()) {
        currentLed = m_ledBuffer.getLength() - 1;
        direction = false;
      }
    } else {
      currentLed--;
      if (currentLed < 0) {
        currentLed = 0;
        direction = true;
      }
    }

    try {
      Thread.sleep(interval); // Sleep for the specified interval
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    return null; // This command never finishes
  }

  /**
   * Executes a wave effect on the LEDs with the specified hue and interval.
   * The brightness of each pixel is calculated using a sine wave.
   * 
   * @param hue      the hue value for the LEDs
   * @param interval the interval in milliseconds between each step of the wave effect
   * @return null, as this command never finishes
   */
  public Command waveEffect(int hue, long interval) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the brightness using a sine wave
          int brightness = (int) ((Math.sin((step + i) / (double) m_ledBuffer.getLength() * 2 * Math.PI) + 1) / 2 * 255);
          m_ledBuffer.setHSV(i, hue, 255, brightness); // Set the HSV value of the pixel
        }
        m_led.setData(m_ledBuffer); // Set the data of the LED buffer
        step++; // Increase the step

        try {
          Thread.sleep(interval); // Sleep for the specified interval
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
        return null; // This command never finishes
    }

  /**
   * Returns a Command object that represents the fire effect.
   * This command sets the HSV values of the LED buffer to create a fire effect.
   * 
   * @return The Command object representing the fire effect N.
   */
  public Command rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) { // For every pixel
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 64); // Set the HSV value of the pixel
    }
    m_rainbowFirstPixelHue += 3; // Increase the hue by 3 to make the rainbow "move"
    m_rainbowFirstPixelHue %= 180; // Keep the hue within the range of 0-180
    m_led.setData(m_ledBuffer); // Set the data of the LED buffer
    return null;
  }
  
  /**
   * Default command that runs the fire effect.
   * 
   * @return The Command object representing the default command.
   */
  public Command getDefaultCommand() {
    return null;
  }
}
 //Hue: 0-180, Saturation: 0-255, Value: 0-255
 //Hue of 180 = Red, 120 = Green, 60 = Blue, 0 = Red

  //setHSV(int index, int hue, int saturation, int value)
  //setRGB(int index, int red, int green, int blue)
  
  /* Hue of:
   * 0 & 180 = Red
   * 15 = Red-Orange
   * 30 = Orange
   * 45 = Orange-Yellow
   * 60 = Yellow
   * 75 = Yellow-Green
   * 90 = Green
   * 105 = Green-Blue
   * 120 = Blue
   * 135 = Blue-Purple
   * 150 = Purple
   * 165 = Purple-Red
   * 180 & 0 = Red
   * Saturation of 0 = White, 255 = Color
   * Value of 0 = Off, 255 = Full Brightness
   */

  //Hues of 0 = Red, 30 = Orange, 60 = Yellow, 90 = Green, 120 = Blue, 150 = Purple, 180 = Red
  //hues of 0-30 or 150-180 are red, 30-60 are orange, 60-90 are yellow, 90-120 are green, 120-150 are blue
  //Hues of 15 is red-orange, 45 is orange-yellow, 75 is yellow-green, 105 is green-blue, 135 is blue-purple, 165 is purple-red



