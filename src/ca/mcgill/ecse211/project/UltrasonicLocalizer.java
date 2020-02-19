package ca.mcgill.ecse211.project;
import static ca.mcgill.ecse211.project.Resources.*; 

import lejos.hardware.Sound;

public class UltrasonicLocalizer{

  /**
   * The initial light sensor value read at the inside of the tile.
   */
  private static double initialLight;
  /**
   * Light sensor values as an array.
   */
  private static float[] lightValue = new float[1];
  /**
   * The distance remembered by the {@code filter()} method.
   */
  private int prevDistance;
  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;
  /**
   * The rotation angle to turn to zero degree heading.
   */
  private double deltaT;
  /**
   * Initializing the rotationAngle.
   */
  private double rotationAngle = 0.0;
  
  /**
   * enumeration for the types of localization.
   *
   */
  
  public enum edgeType {
    FallingEdge, RisingEdge;
  }
  
  /**
   * variable for edge type.
   */
  
  private edgeType type;
  /**
   * data from the US sensor.
   */
  private float[] usData = new float[usSensor.sampleSize()];

  public UltrasonicLocalizer(edgeType type) {
    this.type = type;
  }
  
  /**
   * Method that performs the angle localization.
   */
  public void doLocalization() {
    //initializing variables for two angles
    double angleA, angleB;
    
    //set the rotational speed of the motors
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    
    if (this.type == edgeType.FallingEdge) {
      //rotate until robot faces away from wall
      while (readUsDistance() < WALL_DIST + Resources.errorMargin) {
        turnCtrClockwise();
      }
      //rotate until robot faces wall, record angle
      while (readUsDistance() > WALL_DIST ) {
        turnCtrClockwise();
      }
      //saving the value for the first angle
      angleA = convertAngle(odometer.getXyt()[2]);
      Sound.beepSequenceUp();
      //switch direction and face wall
      while (readUsDistance() < WALL_DIST + Resources.errorMargin) {
        turnClockwise();
      }
      //keep rotating until faces wall
      while (readUsDistance() > WALL_DIST) {
        turnClockwise();
      }
      //stop the robot.
      rightMotor.stop();
      leftMotor.stop();
      Sound.beepSequenceUp();
      // get angle value from odometer
      angleB = convertAngle(odometer.getXyt()[2]);
      // Calculate angle adjustment from angle A or B
      if (angleA > angleB) {
        deltaT = 225 - (angleA + angleB) / 2;
      }
      else {
        deltaT = 45 - (angleA + angleB) / 2;
      }
      // Calculate rotation angle
      rotationAngle = deltaT +  angleB;
      // Turn the robot to face 0 degrees
      leftMotor.rotate(-getDistance(WHEEL_RAD, BASE_WIDTH, rotationAngle-TURNING_ERR_FALLING), true);
      rightMotor.rotate(getDistance(WHEEL_RAD, BASE_WIDTH, rotationAngle-TURNING_ERR_FALLING), false);
       
       
    } else { //RISING EDGE
      
      while (readUsDistance() > WALL_DIST) {
        turnCtrClockwise();
      }
      // keep rotating until the robot no longer sees the wall, then latch the angle
      while (readUsDistance() < WALL_DIST) {
        turnCtrClockwise();
      }

      angleA = odometer.getAngle();
      Sound.beepSequenceUp();
    
      //switch directions and rotate until the robot sees the wall.
      while (readUsDistance() > WALL_DIST) {
        turnClockwise();
      }
       
      // rotate until the robot no longer sees the wall and latch the angle.
      while (readUsDistance() < WALL_DIST) {
        turnClockwise();
      }
      //stop the robot.
      leftMotor.stop(true);
      rightMotor.stop(true);
      // get angle value from odometer
      angleB = odometer.getAngle();
      Sound.beepSequenceUp();
  
      // Calculate angle adjustment from angle A or B
      if (angleA > angleB) {
        deltaT = 225 - (angleA + angleB) / 2 + 90;
      }
      else {
        deltaT = 45 - (angleA + angleB) / 2 + 90;
      }
      // Calculate rotation angle
      rotationAngle = deltaT + odometer.getXyt()[2];//add deltaT and theta value as described above
      //rotate to 0 degree axis
      leftMotor.rotate(-getDistance(WHEEL_RAD, BASE_WIDTH, rotationAngle-TURNING_ERR_RISING), true);
      rightMotor.rotate(getDistance(WHEEL_RAD, BASE_WIDTH, rotationAngle-TURNING_ERR_RISING), false);
    }
  
  }
  
  /**
   * Converts angle from radians to degrees.
   * 
   * @param angle angle in radians
   * @return angle in degrees
   */
  
  private double convertAngle(double angle) {
    return (angle * 180 / Math.PI);
  }

  /**
   * Obtains data from ultrasonic sensor.
   * 
   * @return filtered data
   */
  
  public int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, convert to cm, cast to int, and filter
    return filter((int)(usData[0] * 100.0));
  }
  
  /**
   * Rudimentary filter - toss out invalid samples corresponding to null signal.
   * 
   * @param distance raw distance measured by the sensor in cm
   * @return the filtered distance in cm
   */
  
  int filter(int distance) {
    if (distance >= 255 && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
      // bad value, increment the filter value and return the distance remembered from before
      invalidSampleCount++;
      return prevDistance;
    } else {
      if (distance < 255) {
        // distance went below 255: reset filter and remember the input distance.
        invalidSampleCount = 0;
      }
      prevDistance = distance;
      return distance;
    }
  }
  
  /**
   * Makes robot turn counter clockwise.
   */
  
  private void turnCtrClockwise() {
    leftMotor.backward();
    rightMotor.forward();
  }
  
  /**
   * Makes robot turn clockwise.
   */ 
  
  private void turnClockwise() {
    leftMotor.forward();
    rightMotor.backward();
  }
  
  /**
   * Calculates distance to travel base on radius, width and angle.
   * 
   * @param radius radius of wheel
   * @param width width of robot
   * @param angle angle rotated
   * @return distance to travel
   */
  
  private static int getDistance(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  
  /**
   * Converts degrees to distance.
   * 
   * @param radius radius of the wheel
   * @param distance distance to move
   * @return degrees of rotation of wheel for a distance
   */
  
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  } 
  /**
   * Converts input angle to the total rotation of each wheel needed to rotate 
   * the robot by that angle.
   * 
   * @param angle the input angle
   * @return the wheel rotations necessary to rotate the robot by the angle
   */
  
  public static int AngleToDistance(double angle) {
    return convertDistance(WHEEL_RAD, Math.PI * BASE_WIDTH * angle / 360.0);
  }
  
  /**
   * Turns the robot by a specified angle. Note that this method is different
   * from {@code Navigation.turnTo()}. For example, if the robot is facing 
   * 90 degrees, calling {@code turnBy(90)} will make the robot turn to 
   * 180 degrees, but calling {@code Navigation.turnTo(90)} should do nothing 
   * (since the robot is already at 90 degrees).
   * 
   * @param angle the angle by which to turn, in degrees
   */
  public static void turnBy(double angle) {
    leftMotor.rotate(AngleToDistance(angle), true);
    rightMotor.rotate(-AngleToDistance(angle), false);
  }
  /**
   * Navigates the robot to (1,1) after localization by using Ultrasonic sensor.
   * 
   * @param distX The distance in the x direction to (1,1)
   * @param distY The distance in the y direction to (1,1)
   */
  
  public void usNavigate(double distX, double distY) {
    leftMotor.setSpeed(MOTOR_ROT);
    rightMotor.setSpeed(MOTOR_ROT);
   
    // Move forward in the x direction up to the line.
    leftMotor.rotate(convertDistance(WHEEL_RAD, distX), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, distY), false);
    
    turnBy(96);

    // Move forward in the y direction up to the line.
    leftMotor.rotate(convertDistance(WHEEL_RAD, distX), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, distY), false);
    
    turnBy(-90);
  }
  
  /**
   * Returns light sensor value.
   * 
   * @return light sensor value
   */
  
  public static float getLight() {
    lightSensor.getRedMode().fetchSample(lightValue, 0);
    return lightValue[0] * 100;
  }
  
  /**
   * Navigates the robot to (1,1) after localization by using light sensor.
   * 
   * @param offset distance between sensor and reference point on robot.
   */
  public void lightNavigate(double offset) {
    leftMotor.setSpeed(MOTOR_ROT);
    rightMotor.setSpeed(MOTOR_ROT);

    // Move forward until line is detected.
    while (getColour() < Resources.blackLight) {
      rightMotor.forward();
      leftMotor.forward();
    }
    Sound.beep();
    
    // Move forward by the offset distance.
    leftMotor.rotate(convertDistance(WHEEL_RAD, offset), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, offset), false);
    
    // Turn 90 degrees in the clockwise direction.
    turnBy(90);

    // Move forward until line is detected.
    while (getColour() < Resources.blackLight) {
      rightMotor.forward();
      leftMotor.forward();
    }
    Sound.beep();

    // Move forward by the offset distance.
    leftMotor.rotate(convertDistance(WHEEL_RAD, offset), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, offset), false);
   
    // Turn 90 degrees in the counterclockwise direction.
    turnBy(-88);
 
  }
  
  /**
  * returns the rotationAngle.
  */
  public double getAngle() {
    return this.rotationAngle;
  }
  
  /**
   * Turns around and records x and y distances from the wall.
   */
  
  public double[] recordDistances() {
    //turn to the wall
    leftMotor.setSpeed(MOTOR_HIGH);
    rightMotor.setSpeed(MOTOR_HIGH);
    turnBy(182);
    //record distance from the wall
    double y = readUsDistance();
    //turn to the other wall
    leftMotor.setSpeed(MOTOR_HIGH);
    rightMotor.setSpeed(MOTOR_HIGH);
    turnBy(91);
    //record distance from the wall
    double x = readUsDistance();
    turnBy(93);
    return new double[]  {x , y};
  }
  
   /**
   * Get the color value from the light sensor
   */
  
  public static float getColour() {
    lightSensor.fetchSample(lightValue, 0);
    return lightValue[0];
  }
  
  


  
}
