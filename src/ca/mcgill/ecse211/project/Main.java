package ca.mcgill.ecse211.project;

//static import to avoid duplicating variables and make the code easier to read
import static ca.mcgill.ecse211.project.Resources.*;
import ca.mcgill.ecse211.project.Navigator.mapChoice;
import ca.mcgill.ecse211.project.UltrasonicLocalizer.edgeType;
import lejos.hardware.Button;

/**
 * The main driver class for the lab.
 */
public class Main {
  
  private static double[] corrections = new double[2];
  private static UltrasonicLocalizer ultralocalizer;
  private static Navigator navigator;
   
  /**
   * The main entry point.
   * 
   * @param args not used
   */
  public static void main(String[] args) {
  
    int buttonChoice;
    
    Printer.printerDisplay();   //show initial display options
    buttonChoice = Button.waitForAnyPress(); //wait for a button selection
    //do depending on button choice.
    if (buttonChoice == Button.ID_LEFT) {    
      ultralocalizer = new UltrasonicLocalizer(edgeType.FallingEdge);
    } else if (buttonChoice == Button.ID_RIGHT) {
      ultralocalizer = new UltrasonicLocalizer(edgeType.RisingEdge);
    }
    new Thread(odometer).start();
    //calls the localizer and performs localization
    ultralocalizer.doLocalization();
    //printing the option to light localize
    Printer.typeDisplay();
    buttonChoice = Button.waitForAnyPress();//wait for a button press
    if (buttonChoice == Button.ID_RIGHT) {
      //if light localization, perform light localization
      ultralocalizer.lightNavigate(lightOffset);
    }
    
    //display map types
    Printer.typeMap();
    Button.waitForAnyPress();
    //choosing type of map
    buttonChoice = Button.waitForAnyPress();
    if (buttonChoice == Button.ID_LEFT) {
      navigator = new Navigator(mapChoice.map1);
    } else if (buttonChoice == Button.ID_RIGHT) {
      navigator = new Navigator(mapChoice.map2);
    } else if (buttonChoice == Button.ID_UP) {
      navigator = new Navigator(mapChoice.map3);
    } else if (buttonChoice == Button.ID_DOWN) {
      navigator = new Navigator(mapChoice.map4);
    }
    Button.waitForAnyPress();
    System.exit(0);//exit on press
    
  }


/**
 * Sleeps current thread for the specified duration.
 * 
 * @param duration sleep duration in milliseconds
 */
public static void sleepFor(long duration) {
  try {
    Thread.sleep(duration);
  } catch (InterruptedException e) {
    // There is nothing to be done here
  }
}

}
