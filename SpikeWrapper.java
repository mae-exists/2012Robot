/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

/**
 *
 * @author FRC10
 */
public class SpikeWrapper {
    public Relay relay;
    
    SpikeWrapper(int port) {
        relay = new Relay(port);
    }
    
    public void set(double speed) {
        if(speed==0) {
            relay.set(Relay.Value.kOff);
        } else if (speed > 0) {
            relay.set(Relay.Value.kForward);
        } else if (speed < 0) {
            relay.set(Relay.Value.kReverse);
        } else {
            relay.set(Relay.Value.kOff);
        }
        
    }
}
