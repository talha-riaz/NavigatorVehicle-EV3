package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Main.sleepFor;
import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;

/**
 * This class is used to perform navigation of the robot. 
 * The navigation moves the robot to specified coordinates as defined by a map.
 */
public class Navigator {
  
  private static int map [] [] = new int [5] [2];
  
  public enum mapChoice {
    map1, map2, map3, map4;
  }
  //variable for mapChoice type
  private static mapChoice type;
  
  public Navigator (mapChoice type){
    
    //resetting the odometer..
    odometer.setXyt(30.48, 30.48, 0);
    
    //selection for maps
    if (type == mapChoice.map1) {
      map = Resources.map1;
    } else if (type == mapChoice.map2) {
      map = Resources.map2;
    } else if (type == mapChoice.map3) {
      map = Resources.map3;
    } else if (type == mapChoice.map4) {
      map = Resources.map4;
    }
    //start moving
    startNavigation(map);
    
  }
  
  private static double rotationAngle = 0;
  private double odoValues[] = new double[3];
  
   /**
    * Method which starts the navigation. 
    * Takes points from the map, one by one, and travels to those points
    * @param map
    */
  public void startNavigation(int[][] map) {
    for(int i = 0 ; i < map.length ; i++) {
      travelTo(map[i][0], map[i][1]);
    }
  }
    /**
   * Method takes in the (x,y) coordinates and travels to that point.
   * @param x
   * @param y
   */
  private void travelTo(double x, double y) {
     //get the x and y in cm.
     x = x * TILE_SIZE;
     y = y * TILE_SIZE;
     
     //getting the odometer values
     odoValues = odometer.getXyt();
     
     //getting distance to move from current location (in cm)
     double distX = x - odoValues[0];
     double distY = y - odoValues[1];
     //getting the rotation angle
     double theta = Math.atan2(distX, distY);
     
     //setting the motor speed
     leftMotor.setSpeed(MOTOR_LOW);
     rightMotor.setSpeed(MOTOR_LOW);
     //turn the calculated angle
     turnTo(theta);
     //calculate the straight line distance to move
     double hypotenuse = Math.sqrt(distY*distY  + distX*distX);
     
     leftMotor.setSpeed(MOTOR_HIGH);
     rightMotor.setSpeed(MOTOR_HIGH);
     
     leftMotor.rotate(convertDistance(hypotenuse), true);
     rightMotor.rotate(convertDistance(hypotenuse), false);
  }
    /**
   * Method takes in the angle and makes the robot turn to that angle.
   * @param theta
   */
  private void turnTo(double theta) {
    //calculate rotation angle using the current heading of the robot
    rotationAngle = theta - odoValues[2];
    rotationAngle = Math.toDegrees(rotationAngle);
     //turn the minimum angle to the requried heading
    if(rotationAngle > 180) {
      rotationAngle = 360 - rotationAngle;
    }else if(rotationAngle < -180) {
      rotationAngle = 360 + rotationAngle;
    }
    Sound.beep();

    leftMotor.rotate(convertAngle(rotationAngle),true);
    rightMotor.rotate(-convertAngle(rotationAngle),false);
  }
  
  public static double getRotationAngle() {
    return rotationAngle;
  }
 
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @return the wheel rotations necessary to cover the distance
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
  }

  /**
   * Converts input angle to the total rotation of each wheel needed to rotate 
   * the robot by that angle.
   * 
   * @param angle the input angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  public static int convertAngle(double angle) {
    return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
  }


}
