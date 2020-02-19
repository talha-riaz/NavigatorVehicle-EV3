package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.TEXT_LCD;
import static ca.mcgill.ecse211.project.Resources.odometer;
import java.text.DecimalFormat;

public class Printer {
  
  /**
   * Method to display the options for type of localization.
   *  
   */
  
  public static void printerDisplay() {
    TEXT_LCD.clear();
    TEXT_LCD.drawString("< Left  | Right >", 0, 0);
    TEXT_LCD.drawString("Falling | Rising ", 0, 1);
    TEXT_LCD.drawString(" edge   |  edge  ", 0, 2);
    TEXT_LCD.drawString("        |        ", 0, 3);
    TEXT_LCD.drawString("        |        ", 0, 4);
  
  }
  
  /**
  *  Method to display the option to light localize.
  *  
  */
  
  public static void typeDisplay() {
    TEXT_LCD.clear();
    TEXT_LCD.drawString("<       | Right >", 0, 0);
    TEXT_LCD.drawString("        | Light  ", 0, 1);
    TEXT_LCD.drawString("        |        ", 0, 2);
    TEXT_LCD.drawString("        |        ", 0, 3);
    TEXT_LCD.drawString("        |        ", 0, 4);
  }
  
   /**
  *  Method to display the option to choose a map.
  *  
  */ 
  public static void typeMap() {
    TEXT_LCD.clear();
    TEXT_LCD.drawString("        Up      ", 0, 0);
    TEXT_LCD.drawString("       Map3    ", 0, 1);
    TEXT_LCD.drawString("< Map1     Map2 > ", 0, 2);
    TEXT_LCD.drawString("       Map 4        ", 0, 3);
    TEXT_LCD.drawString("        Down        ", 0, 4);
  }
  
  
}
