// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDsSubSystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  private int step = 0;

  public LEDsSubSystem() {
    m_led = new AddressableLED(1); // Set the LED PWM port to 0

    /* Reuse buffer
    * Default to a length of 60, start empty output
    * Length is expensive to set, so only set it once, then just update data
    */
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
   * Executes a command.
   *
   * @param hue the hue value for the LED color
   * @param value the value for the LED brightness
   * @return the command to set the LEDs to a solid color
   */
  public Command setSolidLED(int hue, int value) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) { // For every pixel
      m_ledBuffer.setHSV(i, hue, 255, value); // Set the HSV value of the pixel
    }
    return null;
  }
  

  /**
   * Executes a fire effect on the LEDs with the specified hue and delay.
   * 
   * @param hue   the hue value for the fire effect
   * @param delay the delay in milliseconds between each step of the effect
   * @return the Command object representing the fire effect
   */
  public Command fireEffect(int hue, long delay) {
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
    return fireEffect(10, 50);
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



