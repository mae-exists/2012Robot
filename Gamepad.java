
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;
import com.sun.squawk.util.MathUtils;

public class Gamepad extends Joystick {
     //Reversal
    boolean reversed = false;

    //Tolerance
   double deadbandTolerance = .05;

    //Axes
   private int joy1XAxis = 1;
   private int joy1YAxis = 2;
   private int joy2XAxis = 3;
   private int joy2YAxis = 4;
   private int hatXAxis = 5;
   private int hatYAxis = 6;

    //Constuctor
   public Gamepad(int port) {
       super(port);
   }

   //Internal Functions
   private double deadband(double value) {
       return(Math.abs(value) < deadbandTolerance)?(0):(value);
   }

   private double getAxis(int axis) {
       return deadband(getRawAxis(axis) * (reversed?-1:1));
   }

   private double magnitude(double x, double y) {
       double degAngle = angle(x,y);
       double largest = 1;

       if (315 < degAngle || degAngle <= 45) {
           largest = Math.sqrt(MathUtils.pow(Math.tan(MathUtils.atan2(x,y)),2) + MathUtils.pow(1,2));
       }
       else if (45 < degAngle && degAngle <= 135) {
           largest = Math.sqrt(MathUtils.pow(1,2) + MathUtils.pow(1 / Math.tan(MathUtils.atan2(x,y)),2));
       }
       else if (135 < degAngle && degAngle <= 225){
           largest = Math.sqrt(MathUtils.pow(-Math.tan(MathUtils.atan2(x,y)),2) + MathUtils.pow(-1,2));
       }
       else if (225 < degAngle && degAngle <= 315){
           largest = Math.sqrt(MathUtils.pow(-1,2) + MathUtils.pow(-1/ Math.tan(MathUtils.atan2(x,y)),2));
       }
       return Math.sqrt(MathUtils.pow(x,2) + MathUtils.pow(y,2)) / largest ;
   }

   private double angle(double x, double y) {
       double angle = (x == 0 && y ==0)?(0):(Math.toDegrees(MathUtils.atan2(x,y)));
       if (angle < 0) {
           angle = angle + 360;
       }
       return angle;
   }

   //Getters
   public double joy1GetX() {
       return getAxis(joy1XAxis);
   }

   public double joy1GetY() {
       return -getAxis(joy1YAxis);
   }

   public double joy2GetX() {
       return getAxis(joy2XAxis);
   }

   public double joy2GetY() {
       return -getAxis(joy2YAxis);
   }

   public double joy1GetMagnitude() {
       return magnitude(joy1GetX(), joy1GetY());
   }

   public double joy2GetMagnitude() {
       return magnitude(joy2GetX(), joy2GetY());
   }

   public double joy1GetAngle() {
       return angle(joy1GetX(), joy1GetY());
   }

   public double joy2GetAngle() {
       return angle(joy2GetX(), joy2GetY());
   }
   public double getHatX() {
       return getAxis(hatXAxis);
   }
   public double getHatY() {
       return -getAxis(hatYAxis);
   }
}
