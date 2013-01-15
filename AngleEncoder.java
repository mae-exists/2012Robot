/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;

public class AngleEncoder extends AnalogChannel {

   //Constants
   private double voltsToDegrees = 72;
   private double offset;

   //Constructor
   public AngleEncoder(int channel, double offsetIn) {
      super(channel);
      offset = offsetIn;
   }

   //Internal Utilities
   private double boundAngle(double angle) {
       if (0 <= angle && angle <= 360){
           return angle;
       }
       if (angle >= 360){
	   angle = angle - 360;
       }
       if (angle < 0){
	   angle = angle + 360;
       }
       return boundAngle(angle);

    }


   //Utilities
   public void zero() {
       offset = super.getVoltage() * voltsToDegrees;
   }

   public void clearOffset() {
       offset = 0;
   }

   public void printInfo() {
       //XXX
       //System.out.println("Voltage = " + super.getVoltage());
       System.out.println("Value = " + this.get());
       System.out.println("Offset = " + offset);
   }

   //Getter
   public double get() {
      return boundAngle(360-(super.getVoltage() * voltsToDegrees - offset));
   }


}
