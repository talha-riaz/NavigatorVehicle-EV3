package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.Odometer;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place for easy access and to avoid
 * cluttering the rest of the codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 */
public class Resources {
  
   /**
   * The type of maps specified in the lab description
   */
  public static final int [] [] map1 = { {1,3} , {2,2} , {3,3} , {3,2} , {2,1} };
  public static final int [] [] map2= { {2,2} , {1,3} , {3,3} , {3,2} , {2,1} };
  public static final int [] [] map3= { {2,1} , {3,2} , {3,3} , {1,3} , {2,2} };
  public static final int [] [] map4= { {1,2} , {2,3} , {2,1} , {3,2} , {3,3} };
  public static final int [] startPt = {1,1};
  
  
  /**
   * The speed at which the robot moves forward in degrees per second.
   */
  public static final int FORWARD_SPEED = 200;
  
  /**
   * The speed at which the robot rotates in degrees per second.
   */
  public static final int ROTATE_SPEED = 150;
  
  /**
   * The tile size in centimeters. Note that 30.48 cm = 1 ft.
   */
  public static final double TILE_SIZE = 30.48;
  /**
   * The error offset when moving to localization by type of localization.
   */
  public static final double TURNING_ERR_FALLING = 0.22;
  public static final double TURNING_ERR_RISING = 8.70;  
  
  /**
   * error margin is introduced to counter the effect of noise.
   */
  public static final int errorMargin = 2;
  /**
   * Robot length in cenitmeters.
   */
  public static final double ROBOT_LENGTH = 5.5;

  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.130;

  /**
   * The robot width in centimeters.
   */
  public static final double BASE_WIDTH = 16.2; //15.9 //16.1

  /**
   * Ideal distance between the sensor and the wall (cm).
   */
  public static final int WALL_DIST = 30;
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_LOW = 50;
  /**
   * Speed of slower rotating wheel (deg/sec).
   */
  public static final int MOTOR_ROT = 100;
  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 200;
  
  /**
   * The limit of invalid samples that we read from the US sensor before assuming no obstacle.
   */
  public static final int INVALID_SAMPLE_LIMIT = 25;
  
  /**
   * The distance between the light sensor and the reference point on the robot.
   */
  public static final double lightOffset = 2.9;
  
  /**
   * Threshold value for light sensor to detect a line.
   */
  public static double blackLight = 13; //13

  
  /**
   * The motor acceleration in degrees per second squared.
   */
  public static final int ACCELERATION = 1500;
  
  
  /**
   * Timeout period in milliseconds.
   */
  public static final int TIMEOUT_PERIOD = 3000;
  
  // Hardware resources
  /**
   * The LCD screen used for displaying text.
   */
  public static final TextLCD TEXT_LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The ultrasonic sensor port setting.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /**
   * The light sensor port setting.
   */
  public static final EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S4);
  
  /**
   * The left motor port setting.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.C);
  
  /**
   * The right motor port setting.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
  
  /**
   * The odometer initialization.
   */
  public static Odometer odometer = Odometer.getOdometer();
}
