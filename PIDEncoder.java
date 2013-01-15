/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class PIDEncoder implements PIDSource {
    public Encoder enc;
    public double encoderRPM = 0;
    public double encoderMarkCounts = 0;
    public double encoderMarkTime = 0;
    
    public double RPMUpdatePeriod = .2;
    
    //Initialize the encoder
    PIDEncoder(int aChannel, int bChannel, boolean reverseDirection, CounterBase.EncodingType encodingType) {
        enc = new Encoder(aChannel, bChannel, reverseDirection, encodingType);
    }
    
    //Return the RPM as pidGet
    public double pidGet() {
            return getRPM();
    }
    public double getRPM() {
        if(Timer.getFPGATimestamp() > RPMUpdatePeriod + encoderMarkTime) {
            double encoderCPS = (enc.get() - encoderMarkCounts) / (Timer.getFPGATimestamp() - encoderMarkTime);
            encoderRPM = (encoderCPS * 60) / 16;
            encoderMarkCounts = enc.get();
            encoderMarkTime = Timer.getFPGATimestamp();
        }
        return encoderRPM;
    }
    
    public double get() {
        return enc.get();
    }
    
    public void start() {
        enc.start();
    }
}