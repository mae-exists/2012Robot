/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.*;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CounterBase.*;
import javax.microedition.io.ServerSocketConnection;
import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;
import java.io.DataInputStream;
import java.io.IOException;

public class ReboundRobot extends IterativeRobot {

    Gamepad driveControl = new Gamepad(1);
    Joystick x52 = new Joystick(2);
    //Gamepad backUpControl = new Gamepad(2);
    //Joystick virtualJoystick = new Joystick(3);

    //Motors
    Jaguar leftMotorOne     = new Jaguar(7);
    Jaguar leftMotorTwo     = new Jaguar(9);
    Jaguar rightMotorOne    = new Jaguar(5);
    Jaguar rightMotorTwo    = new Jaguar(6);
    Jaguar conveyorMotor    = new Jaguar(1);
    Jaguar topRollMotor     = new Jaguar(3);
    Jaguar bottomRollMotor  = new Jaguar(4);
    Jaguar armMotor         = new Jaguar(2);
    Relay rampMotor = new Relay(2);

    //Pneumatics
    Compressor compressor = new Compressor(2,1);
    DoubleSolenoid shiftSolenoid = new DoubleSolenoid(1,2);
    DoubleSolenoid boxSolenoid = new DoubleSolenoid(3,4);
    DoubleSolenoid footSolenoid = new DoubleSolenoid(5,6);
    DoubleSolenoid fireSolenoid = new DoubleSolenoid(8,7);

    //Analog Sensors
    AngleEncoder armEncoder = new AngleEncoder(2, 0.00);
    Gyro gyro = new Gyro(1);

    //Digital Sensors
    PIDEncoder topRollEncoder = new PIDEncoder(11, 12, false, CounterBase.EncodingType.k4X);
    PIDEncoder bottomRollEncoder = new PIDEncoder(9, 13, true, CounterBase.EncodingType.k4X);
    //testing****
//    DigitalInput ten = new DigitalInput(10);
//    DigitalInput nine = new DigitalInput(9);
//    DigitalInput thirteen = new DigitalInput(13);
//    DigitalInput fourteen = new DigitalInput(14);
    ///testing****
    DigitalInput boxOutSwitch = new DigitalInput(6);
    DigitalInput frontBeam = new DigitalInput(4);
    DigitalInput backBeam = new DigitalInput (3);
    //DigitalInput autonomousSelectorOne = new DigitalInput(7);
    //DigitalInput autonomousSelectorTwo = new DigitalInput(8);
    ADXL345_I2C accel = new ADXL345_I2C(1,ADXL345_I2C.DataFormat_Range.k4G);

    //PID Controll
    PIDController topPID = new PIDController(.00008, .000022, 0, topRollEncoder, topRollMotor); //Ki .00008
    PIDController bottomPID = new PIDController(.00008, .000022, 0, bottomRollEncoder, bottomRollMotor);

    //Variables
    double leftSpeed = 0;
    double rightSpeed = 0;
    double armSetpoint = 0;
    double rollerSpd = 0;
    double rollerRPM = 0;
    double conveyorSpd = 0;
    //double topRollSpeed = 0;
    //double bottomRollSpeed = 0;
    boolean teleopInitExecuted = false;
    boolean shiftOn = false;
    boolean manualOverride = false;
    
    //Timing Variables
    double teleopStartTime = 0;
    double fireStartTime = 0;
    boolean fireStart = false;
    boolean footSwitch = false;

    //Autonomous Variables
    int autonomousState = 0;
    double autonomousFireTime = 0;
    double autonomousRampTime = 0;
    double autonomousDriveTime = 0;
    boolean ballOneLeaving = false;
    boolean ballTwoLeaving = false;

    //Send Back Five TCP Values
    int armState = 0;
    int conveyorState = 0;
    int rollerState = 0;
    int unusedState = 0;
    int spare = 0;
    
    //Position Constants
    double armPosBase = 41; //GOOD FOR SPOKANE
    double armPosLow  = armPosBase + 15; //GOOD FOR SPOKANE
    double armPosMed  = armPosBase + 90; //GOOD FOR SPOKANE
    double armPosHigh = armPosBase + 122; //GOOD FOR SPOKANE
    double armPosAuto = armPosBase + 122; //Good for Spokane


    //Speed Constants
    double rollerLowSpd = .8; //FIX ME
    double rollerMedSpd = .4; //FIX ME
    double rollerHighSpd = .8; //FIX ME
    double rollerAutoSpd = .9; //FIX ME
    double conveyorLowSpd = 1; //FIX ME
    double conveyorMedSpd = .8; //FIX ME
    double conveyorHighSpd = .8; //FIX ME
    double autoTopRPMSetpoint = 2330; //Correct?
    double autoBottomRPMSetpoint = 2330; //Correct?
    double autoMedShotRPMSetpoint = 2330;
    double highRPMSetpoint = 1800; //Correct
    
    //Reversal Constants
    double leftOneRevCon = -1;
    double leftTwoRevCon = -1;
    double rightOneRevCon = 1;
    double rightTwoRevCon = 1;
    double conveyorRevCon = 1;
    double topRollRevCon = 1;
    double bottomRollRevCon = 1;
    double reverseActionRevCon = -1;
    double rollerMultiplier = 1;

    //Tolerances
    double armTolerance = 1.5;
    double rollerTolerance = 100;
    double accelSlantedLimit = .5;
    
    //Virtual Joystick Data State Variables
    boolean holdRollerState = false;
    boolean holdConveyorState = false;
  
    //TCP Variables
    boolean TCP = true;
    DataInputStream in;
    ServerSocketConnection ss;
    StreamConnection sock;
  
    //Functions    
    public int nearestArmPos() {
        double angle = armEncoder.get();
        if (angle < (armPosBase + armPosMed)/2) {
            return 0;
        } else if (angle < (armPosMed + armPosHigh)/2) {
            return 2;
        } else if (angle >= (armPosMed + armPosHigh)/2) {
            return 3;
        } else {
            System.out.println("No nearby arm position! Error. ]=");
            return armState;
        }
    }

    public double x52Deadband(double value) {
        if (Math.abs(value) < .1) {return 0;}
        else {return value;}
    }
    
    public void getx52JoystickData() {
        if (x52.getRawButton(1)) {
            armState = nearestArmPos();
            alignArm();
        } else if (x52.getRawButton(6)) {
            armState = 1;
            alignArm();
        } else if (spare != 1) {
            armMotor.set(-x52Deadband(x52.getY()));
            armState = -1;
        }
        
        if(x52.getRawButton(11) || holdConveyorState) { //This needs to change as armState changes even if button not pressed down
            conveyorState = armState;
            holdConveyorState = true;
            //System.out.println("Virtual Button: " + 6);
        } 
        if (x52.getRawButton(12)) { 
            conveyorState = 4;
            holdConveyorState = false;
            //System.out.println("Virtual Button: " + 8);
        }
        if (!x52.getRawButton(12) && !x52.getRawButton(11)) {
            conveyorState = 0;
            holdConveyorState = false;
            //System.out.println("Virtual Button: " + 7);
        }
        
        if(x52.getRawButton(9) || holdRollerState) { //This needs to change as armState changes even if button not pressed down
            rollerState = armState;
            holdRollerState = true;
            //System.out.println("Virtual Button: " + 9);
        }
        if(x52.getRawButton(10)) {
            rollerState = 4;
            holdRollerState = false;
            //System.out.println("Virtual Button: " + 10);
        }
        if(!x52.getRawButton(9) && !x52.getRawButton(10)){
            rollerState = 0;
            holdRollerState = false;
            //System.out.println("Virtual Button: " + 11);
        }   
        
        if(x52.getRawButton(5)) {
            spare = 1;
            //System.out.println("Virtual Button: " + 12);
        } else {
            spare = 0;
        }
        
        if(x52.getRawButton(7)) {
            fireSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            fireSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    
        if (!(x52.getRawButton(2))) {footSwitch = false;}
        
        if (x52.getRawButton(2) && !footSwitch) {
            if (footSolenoid.get() == DoubleSolenoid.Value.kForward) {
                footSolenoid.set(DoubleSolenoid.Value.kReverse);
            } else if (footSolenoid.get() == DoubleSolenoid.Value.kReverse) {
                footSolenoid.set(DoubleSolenoid.Value.kForward);
            }
            footSwitch = true;
        }
    }

    public void alignRollers() {
        if (rollerState == 4) {
            if (armState == 0 || armState == 4) {
                rollerSpd = 0;
            } else if (armState == 1) {
                rollerSpd = rollerLowSpd * reverseActionRevCon;
            } else if (armState == 2) {
                rollerSpd = rollerMedSpd * reverseActionRevCon;
            } else if (armState == 3) {
                rollerSpd = rollerHighSpd * reverseActionRevCon;
            } else if (armState == -1) {
                rollerSpd = rollerHighSpd * reverseActionRevCon;
            }
        } else if (rollerState == 3) {
            rollerSpd = rollerHighSpd;
        } else if (rollerState == 2) {
            rollerSpd = rollerMedSpd;
        } else if (rollerState == 1) {
            rollerSpd = rollerLowSpd;
        } else if (rollerState == -1) {
            rollerSpd = rollerHighSpd;
        } else {
            rollerSpd = 0;
        }

        rollerSpd = rollerSpd * rollerMultiplier;

        if (rollerSpd > 1) {
            rollerSpd = 1;
        } else {
            topRollMotor.set(rollerSpd * topRollRevCon);
            bottomRollMotor.set(rollerSpd * bottomRollRevCon);
        }
    }

    public void alignConveyor() {
        if (conveyorState == 4) {
            if (armState == 0 || armState == 4) {
                conveyorSpd = 0;
            } else if (armState == 1) {
                conveyorSpd = conveyorLowSpd * reverseActionRevCon;
            } else if (armState == 2) {
                conveyorSpd = conveyorMedSpd * reverseActionRevCon;
            } else if (armState == 3) {
                conveyorSpd = conveyorHighSpd * reverseActionRevCon;
            } else if (armState == -1) {
                conveyorSpd = conveyorHighSpd * reverseActionRevCon;
            }
        } else if (conveyorState == 3) {
            conveyorSpd = conveyorHighSpd;
        } else if (conveyorState == 2) {
            conveyorSpd = conveyorMedSpd;
        } else if (conveyorState == 1) {
            conveyorSpd = conveyorLowSpd;
        } else if (conveyorState == -1) {
            conveyorSpd = conveyorHighSpd;
        } else {
            conveyorState = 0;
            conveyorSpd = 0;
        }
        
        conveyorMotor.set(conveyorSpd * conveyorRevCon);
    }

    public void alignArm() {
        if (armState == 5) {
            armSetpoint = armPosAuto;
            boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (armState == 4) {
            armSetpoint = armEncoder.get();
            //do we need to set the setpoint to the current position here
        } else if (armState == 3) {
            armSetpoint = armPosHigh;
            boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (armState == 2) {
            armSetpoint = armPosMed;
            boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (armState == 1) {
            armSetpoint = armPosLow;
        } else if (armState == 0) {
            armSetpoint = armPosBase;
            boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (armState == -1) {
            //boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        double armError = armEncoder.get() - armSetpoint;
        if (Math.abs(armError) <= armTolerance) {
            armMotor.set(0);
            if (armState == 1) {
                boxSolenoid.set(DoubleSolenoid.Value.kForward);
            }
        } else if (armError < 0 && armError > -20 && !boxOutSwitch.get()) {
            armMotor.set(.5);
        } else if (armError <= -20 && !boxOutSwitch.get()) {
            armMotor.set(.7);
        } else if (armError >= 0 && armError < 20 && !boxOutSwitch.get()) {
            armMotor.set(-.3);
        } else if (armError >= 20 && !boxOutSwitch.get()) {
            armMotor.set(-.6);
        }
        
        if(armEncoder.get() < armPosLow && armError < 0) { //Extra oopmh when going up from base
            armMotor.set(1.4 * armMotor.get());
        }
    }

    public void updateDashboard() {
        Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        lowDashData.addDouble(rollerRPM);//accel.getAcceleration(ADXL345_I2C.Axes.kZ));
        lowDashData.addInt(2);
        lowDashData.addDouble(Timer.getFPGATimestamp() - teleopStartTime);
        lowDashData.addDouble(topRollEncoder.getRPM());
        lowDashData.addDouble(bottomRollEncoder.getRPM());
        lowDashData.addBoolean(frontBeam.get());
        lowDashData.addDouble(topPID.getError());
        lowDashData.addDouble(bottomPID.getError());
        lowDashData.addBoolean(backBeam.get());
        lowDashData.commit();
    }

    public void robotInit() {
        Watchdog.getInstance().setEnabled(false);
        compressor.start();
        shiftSolenoid.set(DoubleSolenoid.Value.kReverse);
        boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        footSolenoid.set(DoubleSolenoid.Value.kReverse);
        fireSolenoid.set(DoubleSolenoid.Value.kReverse);
        
        
        topRollEncoder.start();
        bottomRollEncoder.start();

        topPID.setInputRange(0,3000);
        topPID.setOutputRange(0, 1);
        topPID.setTolerance(1.5);
        topPID.disable();
        bottomPID.setInputRange(0,3000);
        bottomPID.setOutputRange(0, 1);
        bottomPID.setTolerance(1.5);
        bottomPID.disable();
    }
    
    public void autonomousPeriodic() {
        final int INITIALIZE = 0;
        final int POSITION = 1;
        final int FIRE = 2;
        final int TUCK = 3;
        final int COMPLETE = 4;
        
        updateDashboard();
        
        if (autonomousState == INITIALIZE) {
            armMotor.set(0);
            conveyorMotor.set(0);
            topRollMotor.set(0);
            bottomRollMotor.set(0);
            boxSolenoid.set(DoubleSolenoid.Value.kReverse);
            fireSolenoid.set(DoubleSolenoid.Value.kReverse);
            autonomousState++;
        } else if (autonomousState == POSITION) {
            armState = 3;
            alignArm();
            leftMotorOne.set(.4 * leftOneRevCon);
            rightMotorOne.set(.4 * rightOneRevCon);
            leftMotorTwo.set(.4 * leftTwoRevCon);
            rightMotorTwo.set(.4 * rightTwoRevCon);
            if (autonomousDriveTime == 0) {autonomousDriveTime = Timer.getFPGATimestamp();}
            if ((Math.abs(armEncoder.get() - armPosHigh) <= armTolerance) && Timer.getFPGATimestamp() > (autonomousDriveTime + 3)) {
                leftMotorOne.set(0);
                rightMotorOne.set(0);
                leftMotorTwo.set(0);
                rightMotorTwo.set(0);
                autonomousState++;
            }
        } else if (autonomousState == FIRE) {
            alignArm();
            PIDFire(highRPMSetpoint, highRPMSetpoint);
            if(!frontBeam.get()) {
                ballOneLeaving = true;
            }
            if(ballOneLeaving && frontBeam.get()) {
                System.out.println("Clear to load next ball");
                fireSolenoid.set(DoubleSolenoid.Value.kForward);
            }
            if(fireSolenoid.get() == DoubleSolenoid.Value.kForward && !frontBeam.get()) {
                System.out.println("Front Beam Crossed AGAIN");
                ballTwoLeaving = true;
            }
            if(ballTwoLeaving == true && frontBeam.get()){
                System.out.println("I shouldn't be running until 2 balls have been fired.");
                topRollMotor.set(0);
                bottomRollMotor.set(0);
                conveyorMotor.set(0);
                topPID.disable();
                bottomPID.disable();
                autonomousState++;   
            }
        } else if (autonomousState == TUCK) {
            armState = 0;
            alignArm();
            if (autonomousRampTime == 0) {autonomousRampTime = Timer.getFPGATimestamp();}
            if (Timer.getFPGATimestamp() > (autonomousRampTime + 2)) {
                leftMotorOne.set(-.5);
                rightMotorOne.set(-.5);
                leftMotorTwo.set(-.5);
                rightMotorOne.set(-.5);
            } else {
                leftMotorOne.set(0);
                rightMotorOne.set(0);
                leftMotorTwo.set(0);
                rightMotorTwo.set(0);
            }
            if (Math.abs(armEncoder.get() - armPosAuto) <= armTolerance) {
                leftMotorOne.set(0);
                rightMotorOne.set(0);
                leftMotorTwo.set(0);
                rightMotorTwo.set(0);
                topRollMotor.set(0);
                bottomRollMotor.set(0);
                conveyorMotor.set(0);
                topPID.disable();
                bottomPID.disable();
                armMotor.set(0);
                autonomousState++;
            }
        } else if (autonomousState == COMPLETE) {
            //Ask Jared what happens now.
        }
    }
    
    public void autonomousPeriodicOld() {
        if (true) { //formerly autonomous selector?
            autonomousPeriodic();
        } else {
        //Static Stuff
        final int INITIALIZE = 0;
        final int RAISEARM = 1;
        final int FIRE = 2;
        final int LOWERARM = 3;
        final int BACKUP = 4;
        final int COMPLETE = 5;
        final int ARMLOW = 6;
        final int FEED = 7;
        //Static Stuff

        updateDashboard();

        if(autonomousState == INITIALIZE) {
                System.out.println("(init) Arm Encoder: " + armEncoder.get());
                armMotor.set(0);
                conveyorMotor.set(0);
                topRollMotor.set(0);
                bottomRollMotor.set(0);
                if(true) {
                    autonomousState++;
                }
        } else if(autonomousState == RAISEARM) {
                armState = 5;
                alignArm();
                if(Math.abs(armEncoder.get() - armPosAuto) <= armTolerance) {
                    System.out.println("Arm Encoder: " + armEncoder.get());
                    System.out.println("Arm Position: " + armPosAuto);
                    System.out.println("Arm Tolerance: " + armTolerance);
                    autonomousState++;
                }

                if(!topPID.isEnable()) {topPID.enable();}
                if(!bottomPID.isEnable()) {bottomPID.enable();}
                topPID.setSetpoint(autoTopRPMSetpoint);
                bottomPID.setSetpoint(autoBottomRPMSetpoint);

        } else if(autonomousState ==  FIRE) {
                alignArm();
                if(autonomousFireTime == 0) {
                    autonomousFireTime = Timer.getFPGATimestamp();
                }
                if(Timer.getFPGATimestamp() > autonomousFireTime + 12) { //formerly +6
                    topRollMotor.set(0);
                    bottomRollMotor.set(0);
                    conveyorMotor.set(0);
                    topPID.disable();
                    bottomPID.disable();
                    autonomousState++;
                } else if(Timer.getFPGATimestamp() > autonomousFireTime + 1) {
                    PIDFire(autoTopRPMSetpoint, autoBottomRPMSetpoint);
                }

        } else if(autonomousState == LOWERARM) {
                armState = 0;
                alignArm();
                if(Math.abs(armEncoder.get() - armSetpoint) <= armTolerance) {
                    autonomousState++;
                }
        } else if(autonomousState == BACKUP) {
                alignArm();
                shiftSolenoid.set(DoubleSolenoid.Value.kForward);
                if(autonomousRampTime == 0) {
                    autonomousRampTime = Timer.getFPGATimestamp();
                }
                if (Timer.getFPGATimestamp() > autonomousRampTime + .5) {
                    leftMotorOne.set(-.55 * leftOneRevCon);
                    rightMotorOne.set(-.5 * rightOneRevCon);
                    leftMotorTwo.set(-.55 * leftTwoRevCon);
                    rightMotorTwo.set(-.5 * rightTwoRevCon);
                }  

                if(Timer.getFPGATimestamp() > autonomousRampTime + 2) {
                    rampMotor.set(Relay.Value.kOff);
                    if (Timer.getFPGATimestamp() > autonomousRampTime + 5.3) {//(!limitAccelerometer.get()) {
                        System.out.println("Limit Aceelerometer Is False");
                        leftMotorOne.set(0);
                        rightMotorOne.set(0);
                        leftMotorTwo.set(0);
                        rightMotorTwo.set(0);
                        rampMotor.set(Relay.Value.kOff);
                        autonomousState++;
                    }
                } else {
                    rampMotor.set(Relay.Value.kReverse);
                }

        } else if(autonomousState == COMPLETE) {
            //Stuff
        } else if(autonomousState == ARMLOW) {
            //Arm to gather position
        }
            //updateDashboard();

        } 
    }
    
    public void autonomousDisabled() {
        updateDashboard();
    }

    public void teleopInit() {
        teleopStartTime = Timer.getFPGATimestamp();
        if(!teleopInitExecuted) {
            //openTCPConnection();
        }
        teleopInitExecuted = true;
    }
    
    public void teleopDisabled() {
        updateDashboard();
    }

    public void PIDFire(double topRPM, double bottomRPM) {
        if(!topPID.isEnable()) {topPID.enable();}
        if(!bottomPID.isEnable()) {bottomPID.enable();}
        
        System.out.println("Top Setpoint: " + topRPM);
        System.out.println("Top Speed" + topRollEncoder.getRPM());
        System.out.println("Bottom Setpoint: " + bottomRPM);
        System.out.println("Bottom Speed" + bottomRollEncoder.getRPM());
        
        topPID.setSetpoint(topRPM);
        bottomPID.setSetpoint(bottomRPM);
        
        if (topPID.onTarget() && bottomPID.onTarget()) {
            conveyorMotor.set(conveyorHighSpd * conveyorRevCon);
        } else {
            conveyorMotor.set(0);
        }

//        if (topPID.onTarget() && bottomPID.onTarget() && !fireStart) {
//            conveyorMotor.set(conveyorHighSpd * conveyorRevCon);
//            fireSolenoid.set(DoubleSolenoid.Value.kReverse);
//            fireStartTime = Timer.getFPGATimestamp();
//            fireStart = true;
//        } else if (topPID.onTarget() && bottomPID.onTarget() && fireStart) {
//            conveyorMotor.set(conveyorHighSpd * conveyorRevCon);
//            if (Timer.getFPGATimestamp() > fireStartTime + 1 && Timer.getFPGATimestamp() < fireStartTime + 1.5) {
//                fireSolenoid.set(DoubleSolenoid.Value.kForward);
//            } else if (Timer.getFPGATimestamp() > fireStartTime + 3 && Timer.getFPGATimestamp() < fireStartTime + 3.5) {
//                fireSolenoid.set(DoubleSolenoid.Value.kForward);
//            } else {
//                fireSolenoid.set(DoubleSolenoid.Value.kReverse);
//            }
//        } else if (fireStart) {
//            conveyorMotor.set(0);
//            fireStartTime = Timer.getFPGATimestamp();
//            fireSolenoid.set(DoubleSolenoid.Value.kReverse);
//        } else {
//            fireStartTime = 0;
//            conveyorMotor.set(0);
//            fireSolenoid.set(DoubleSolenoid.Value.kReverse);
//        }
    }

    public void teleopPeriodic() {
        updateDashboard();
        //getVirtualJoystickData();//getTCPData();
        getx52JoystickData();
        
        //Drive Function
        leftSpeed = driveControl.joy1GetY();
        rightSpeed = driveControl.joy2GetY();

        leftMotorOne.set(leftOneRevCon * leftSpeed);
        leftMotorTwo.set(leftTwoRevCon * leftSpeed);
        rightMotorOne.set(rightOneRevCon * rightSpeed);
        rightMotorTwo.set(rightTwoRevCon * rightSpeed);

        if (driveControl.getRawButton(5) && driveControl.getRawButton(6)) {
            shiftSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {shiftSolenoid.set(DoubleSolenoid.Value.kReverse);}

        if (driveControl.getRawButton(7)) {rampMotor.set(Relay.Value.kForward);
        } else if (driveControl.getRawButton(8)) {rampMotor.set(Relay.Value.kReverse);
        } else {rampMotor.set(Relay.Value.kOff);}
        
//        if (!(driveControl.getRawButton(4) && driveControl.getRawButton(3))) {footSwitch = false;}
//        
        if (driveControl.getRawButton(1)) {
            boxSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else if (driveControl.getRawButton(2)) {
            boxSolenoid.set(DoubleSolenoid.Value.kForward);
        }
//        
//        if (driveControl.getRawButton(3) && driveControl.getRawButton(4) && !footSwitch) {
//            if (footSolenoid.get() == DoubleSolenoid.Value.kForward) {
//                footSolenoid.set(DoubleSolenoid.Value.kReverse);
//            } else if (footSolenoid.get() == DoubleSolenoid.Value.kReverse) {
//                footSolenoid.set(DoubleSolenoid.Value.kForward);
//            }
//            
//            footSwitch = true;
//        }

        if (driveControl.getRawButton(9)) {
            System.out.println("Angle Reading: " + armEncoder.get());
            System.out.println("Accelerometer Z axis(g): " + accel.getAcceleration(ADXL345_I2C.Axes.kZ));
            System.out.println("Top Encoder Counts: " + topRollEncoder.get());
            System.out.println("Bottom Encoder Counts: " + bottomRollEncoder.get());
            System.out.println("Top Encoder RPM: " + topRollEncoder.getRPM());
            System.out.println("Bottom Encoder RPM: " + bottomRollEncoder.getRPM());
            System.out.println("Arm State: " + armState);
            System.out.println("Front Beam: " + frontBeam.get());
            System.out.println("The code is hungry.");
        }
        
        //Other Stuff
//        if(Timer.getFPGATimestamp() > .2 + encoderRPMUpdateMarkTime) {
//            updateTopEncoderRPM();
//            updateBottomEncoderRPM();
//            encoderRPMUpdateMarkTime = Timer.getFPGATimestamp();
//        }

        //Second Drive
//        if (backUpControl.getRawButton(10)) {
//            manualOverride = true;
//        } else if (backUpControl.getRawButton(9)) {
//            manualOverride = false;
//        }

        //if (!manualOverride) {
        //    alignArm();            
            
//            if (backUpControl.getRawButton(2)) {
//                rollerRPM = rollerRPM - 100;
//            } else if (backUpControl.getRawButton(1)) {
//                rollerRPM = rollerRPM + 100;
//            }

            //topPID.setSetpoint(rollerRPM * topRollRevCon);
            //bottomPID.setSetpoint(rollerRPM * bottomRollRevCon);
//
//          double topRPMError = rollerRPM - topRollEncoder.getRPM();
//          double bottomRPMError = rollerRPM - bottomRollEncoder.getRPM();
//

            //Ball Shooting Function
            if (spare == 1 && (armState == 2 || armState == 3 || armState == -1)) {
                PIDFire(highRPMSetpoint, highRPMSetpoint);
            } else {
                fireStart = false;
                if(topPID.isEnable()) {topPID.disable();}
                if(bottomPID.isEnable()) {bottomPID.disable();}
                alignConveyor();
                alignRollers();
                //topPID.setSetpoint(rollerRPM * topRollRevCon);
                //bottomPID.setSetpoint(rollerRPM * bottomRollRevCon);
            }
            
            //Don't Pull Out Wires
            if(armEncoder.get() > (armPosLow + 15)) {
                boxSolenoid.set(DoubleSolenoid.Value.kReverse);
            }
            
            
//        } else {
//            if(topPID.isEnable()) {topPID.disable();}
//            if(bottomPID.isEnable()) {bottomPID.disable();}            
//            
//            armMotor.set(backUpControl.joy1GetY());
//
//            if (backUpControl.getRawButton(4)) {
//                conveyorMotor.set(1 * conveyorRevCon);
//            } else if (backUpControl.getRawButton(3)) {
//                conveyorMotor.set(-1 * conveyorRevCon);
//            } else {
//                conveyorMotor.set(0);
//            }

//            if (backUpControl.getRawButton(2)) {
//                rollerSpd = rollerSpd - .05;
//            } else if (backUpControl.getRawButton(1)) {
//                rollerSpd = rollerSpd + .05;
//            }
//            
//            topRollMotor.set(rollerSpd * topRollRevCon);
//            bottomRollMotor.set(rollerSpd * bottomRollRevCon);
//
//            if(backUpControl.getRawButton(7)) {
//                pistonSolenoid.set(DoubleSolenoid.Value.kForward);
//            } else if(backUpControl.getRawButton(8)) {
//                pistonSolenoid.set(DoubleSolenoid.Value.kReverse);
//            }
//        }
    }
    //Stuff
    {
    //    public void getVirtualJoystickData() {
//        if(virtualJoystick.getRawButton(1)) {
//            armState = 0;
//            //System.out.println("Virtual Button: " + 1);
//        }
//        if(virtualJoystick.getRawButton(2)) {
//            armState = 1;
//            //System.out.println("Virtual Button: " + 2);
//        }
//        if(virtualJoystick.getRawButton(3)) {
//            armState = 2;
//            //System.out.println("Virtual Button: " + 3);
//        }
//        if(virtualJoystick.getRawButton(4)) {
//            armState = 3;
//            //System.out.println("Virtual Button: " + 4);
//        }
//        if(virtualJoystick.getRawButton(5)) {
//            armState = 4;
//            //System.out.println("Virtual Button: " + 5);
//        }
//        if(virtualJoystick.getRawButton(6) || holdConveyorState) { //This needs to change as armState changes even if button not pressed down
//            conveyorState = armState;
//            holdConveyorState = true;
//            //System.out.println("Virtual Button: " + 6);
//        }
//        if(virtualJoystick.getRawButton(7)) {
//            conveyorState = 0;
//            holdConveyorState = false;
//            //System.out.println("Virtual Button: " + 7);
//        }   
//        if(virtualJoystick.getRawButton(8)) { 
//            conveyorState = 4;
//            holdConveyorState = false;
//            //System.out.println("Virtual Button: " + 8);
//        }   
//        if(virtualJoystick.getRawButton(9) || holdRollerState) { //This needs to change as armState changes even if button not pressed down
//            rollerState = armState;
//            holdRollerState = true;
//            //System.out.println("Virtual Button: " + 9);
//        }   
//        if(virtualJoystick.getRawButton(10)) {
//            rollerState = 4;
//            holdRollerState = false;
//            //System.out.println("Virtual Button: " + 10);
//        }   
//        if(virtualJoystick.getRawButton(11)) {
//            rollerState = 0;
//            holdRollerState = false;
//            //System.out.println("Virtual Button: " + 11);
//        }   
//        if(virtualJoystick.getRawButton(12)) {
//            spare = 1;
//            //System.out.println("Virtual Button: " + 12);
//        } else {
//            spare = 0;
//        }
//    }
    //        if (driveControl.getRawButton(5) && driveControl.getRawButton(6)) {
//            shiftSolenoid.set(DoubleSolenoid.Value.kForward);
//            shiftOn = true;
//        } else if (shiftOn) {
//            shiftSolenoid.set(DoubleSolenoid.Value.kReverse);
//            shiftOn = false;
//        }

        //    public boolean robotAtAngle() {
//        if (accel.getAcceleration(ADXL345_I2C.Axes.kZ) < .997) {
//            return true;
//        } else {
//            return false;
//        }
//    }

    //Flags & Delays
    //double shootDelay = 5;
    //double flagTarget = 0;
    //boolean shootFlag = false;
    //boolean ballSeen = false;
        
        //    public void initBottomEncoder() {
//        bottomEncoder.setUpSource(bottomEncoderSwitch);
//        bottomEncoder.clearDownSource();
//        bottomEncoder.start();
//    }

//    public void getRPM() {
//        //Period Calculations
//        bottomPeriod = bottomEncoder.getPeriod();
//        bottomTotalCounts = bottomEncoder.get();
//        if(bottomPeriod != 0) {
//            bottomPeriodRate = (1 / bottomEncoder.getPeriod()) * 60;
//        } else {
//            bottomPeriodRate = 0;
//        }
//
//        //Derived Calculations
//        double MarkTime = Timer.getFPGATimestamp();
//        double MarkCounts = bottomEncoder.get();
//        double deltaT = MarkTime - lastMarkTime;
//        double deltaC = MarkCounts - lastMarkCounts;
//        double CperS = deltaC/deltaT;
//        bottomDerivedRate = 60*CperS;
//        lastMarkTime = MarkTime;
//        lastMarkCounts = MarkCounts;
//
//    }

//    public void determineCountedRate() {
//        //double count = bottomEncoder.get();
//        double time = Timer.getFPGATimestamp();
//        double recentCounts = count - storedCounts;
//        double recentTime = time - storedTime;
//        double recentTrueCount = trueCount - storedTrueCount;
//        functionCalled++;
//        if (recentCounts > 0) {
//            if (!edgeSeen) {
//                trueCount++;
//            }
//            edgeSeen = true;
//        } else {
//            edgeSeen = false;
//        }
//        storedCounts = count;
//        if(recentTime > .1) {
//            storedTime = time;
//            bottomRate = recentTrueCount/(recentTime / 60);
//        }
//    }
        
    //    public void alignPIDStates() {
//        if (rollerState == 4) {
//            if (armState == 0 || armState == 4) {
//                rollerSpd = 0;
//            } else if (armState == 1) {
//                rollerSpd = rollerLowSpd * reverseActionRevCon;
//            } else if (armState == 2) {
//                rollerSpd = rollerMedSpd * reverseActionRevCon;
//            } else if (armState == 3) {
//                rollerSpd = rollerHighSpd * reverseActionRevCon;
//            }
//        } else if (rollerState == 3) {
//            rollerSpd = rollerHighSpd;
//        } else if(rollerState == 2) {
//            rollerSpd = rollerMedSpd;
//        } else if(rollerState == 1) {
//            rollerSpd = rollerLowSpd;
//        } else {
//            rollerSpd = 0;
//        }
//
//        topPID.setSetpoint(rollerSpd * topRollRevCon);
//        bottomPID.setSetpoint(rollerSpd * bottomRollRevCon);
//
//        //rollerSpd = rollerSpd * rollerMultiplier;
//
////        if (rollerSpd >= 1) {
////            rollerSpd = .99;
////        } else {
////            //topPID.setSetpoint(rollerSpd * topRollRevCon);
////            //bottomPID.setSetpoint(rollerSpd * bottomRollRevCon);
////        }
//
//    }

    //double rampLowerTarget = 0;
    //double driveTarget = 0;
    //boolean autonomousCompleted = false;
    //boolean rampHit = false;
    //boolean armLifted = false;
    //boolean rampLowered = false;
    //boolean rampFlag = false;
    //boolean ballsDropped = false;
    //boolean driveFlag = false;
    //double rampLowerDelay = 1.5;
    //double accelImpactLimit = .5;
    //    public void autonomousPeriodic() {
//        updateDashboard();
//
//        if(autonomousCompleted) {
//            return;
//        }
//
//        //Lift Arm To Top
//        armSetpoint = armPosTop;
//
//        double armError = armEncoder.get() - armSetpoint;
//
//        if (Math.abs(armError) <= armTolerance) {
//            armLifted = true;
//        } else if (armError < 0 && armError > -10) {
//            armMotor.set(.3);
//        } else if (armError <= -10) {
//            armMotor.set(.5);
//        } else if (armError >= 0 && armError < 10) {
//            armMotor.set(-.15);
//        } else if (armError >= 10) {
//            armMotor.set(-.25);
//        }
//
//        //Back Up to Ramp
//        if (accel.getAcceleration(ADXL345_I2C.Axes.kY) < accelImpactLimit && !rampHit) {
//            shiftSolenoid.set(DoubleSolenoid.Value.kForward);
//            leftMotorOne.set(-1 * leftOneRevCon);
//            rightMotorOne.set(-1 * rightOneRevCon);
//            leftMotorTwo.set(-1 * leftTwoRevCon);
//            rightMotorTwo.set(-1 * leftOneRevCon);
//        } else {
//            rampHit = true;
//        }
//
//        if (armLifted && !rampLowered) {
//            rampFlag = true;
//            rampMotor.set(1);
//            rampLowerTarget = Timer.getFPGATimestamp() + rampLowerDelay;
//        }
//
//        if (rampFlag && rampLowerTarget < Timer.getFPGATimestamp()) {
//            rampFlag = false;
//            rampMotor.set(0);
//        }
//
//        if (rampHit && !rampLowered) {
//            shiftSolenoid.set(DoubleSolenoid.Value.kForward);
//            leftMotorOne.set(.5 * leftOneRevCon);
//            rightMotorOne.set(.5 * rightOneRevCon);
//            leftMotorTwo.set(.5 * leftTwoRevCon);
//            rightMotorTwo.set(.5 * leftOneRevCon);
//        }
//
//        //Back Up Onto Ramp
//        if (rampHit && rampLowered && !robotAtAngle() && !ballsDropped) {
//            shiftSolenoid.set(DoubleSolenoid.Value.kForward);
//            leftMotorOne.set(-.75 * leftOneRevCon);
//            rightMotorOne.set(-.75 * rightOneRevCon);
//            leftMotorTwo.set(-.75 * leftTwoRevCon);
//            rightMotorTwo.set(-.75 * leftOneRevCon);
//        }
//
//        if (rampHit && rampLowered && robotAtAngle()) {
//            ballsDropped = true;
//            driveTarget = Timer.getFPGATimestamp() + 1.5;
//        }
//
//        if (ballsDropped) {
//            driveFlag = true;
//            shiftSolenoid.set(DoubleSolenoid.Value.kForward);
//            leftMotorOne.set(.75 * leftOneRevCon);
//            rightMotorOne.set(.75 * rightOneRevCon);
//            leftMotorTwo.set(.75 * leftTwoRevCon); 
//            rightMotorTwo.set(.75 * leftOneRevCon);
//            topPID.setSetpoint(topRollAutoSpd * bottomRollRevCon);
//            bottomPID.setSetpoint(bottomRollAutoSpd * bottomRollRevCon);
//            topPID.enable();
//            bottomPID.enable();
//            conveyorMotor.set(-.7 * conveyorRevCon);
//        }
//
//        if(driveFlag && Timer.getFPGATimestamp() > driveTarget && !robotAtAngle()) {
//            leftMotorOne.set(0 * leftOneRevCon);
//            rightMotorOne.set(0 * rightOneRevCon);
//            leftMotorTwo.set(0 * leftTwoRevCon);
//            rightMotorTwo.set(0 * leftOneRevCon);
//        }
//    }
    
    //            topRollSpeed += topRPMError * (1 / 2800);
//            bottomRollSpeed += bottomRPMError * (1 / 2800);
//            
//            topRollMotor.set(topRollSpeed * topRollRevCon);
//            bottomRollMotor.set(bottomRollSpeed * bottomRollRevCon);
            
            //2800 max rpm
    
    //    public void updateTopEncoderRPM () {
//        double topEncoderCPS = (topRollEncoder.get() - topEncoderMarkCounts) / (Timer.getFPGATimestamp() - topEncoderMarkTime);
//        topEncoderRPM = (topEncoderCPS * 60) / 16;
//        topEncoderMarkCounts = topRollEncoder.get();
//        topEncoderMarkTime = Timer.getFPGATimestamp();
//    }
//    
//    public void updateBottomEncoderRPM () {
//        double bottomEncoderCPS = (bottomRollEncoder.get() - bottomEncoderMarkCounts) / (Timer.getFPGATimestamp() - bottomEncoderMarkTime);
//        bottomEncoderRPM = (bottomEncoderCPS * 60) / 16;
//        bottomEncoderMarkCounts = bottomRollEncoder.get();
//        bottomEncoderMarkTime = Timer.getFPGATimestamp();
//    }
    
//    public void alignPIDStates() {
//        if (rollerState == 4) {
//            if (armState == 0 || armState == 4) {
//                rollerSpd = 0;
//            } else if (armState == 1) {
//                rollerSpd = rollerLowSpd * reverseActionRevCon;
//            } else if (armState == 2) {
//                rollerSpd = rollerMedSpd * reverseActionRevCon;
//            } else if (armState == 3) {
//                rollerSpd = rollerHighSpd * reverseActionRevCon;
//            }
//        } else if (rollerState == 3) {
//            rollerSpd = rollerHighSpd;
//        } else if(rollerState == 2) {
//            rollerSpd = rollerMedSpd;
//        } else if(rollerState == 1) {
//            rollerSpd = rollerLowSpd;
//        } else {
//            rollerSpd = 0;
//        }
//
//        topPID.setSetpoint(rollerSpd * topRollRevCon);
//        bottomPID.setSetpoint(rollerSpd * bottomRollRevCon);
//    }
    }
    //TCP Stuff
    {
    //    public int eightBitToInt(String s) {
//        int output = 0;
//        for(int i = 0; i <= 7; i++) {
//            output += MathUtils.pow(2,i)*Integer.parseInt(s.substring(i,i+1));
//        }
//        return output;
//        //formerly known as eightBitBinaryStringToInt
//    }
//
//    public void openTCPConnection() {
//        if(TCP) {
//            try {
//                System.out.println("Opening server socket connection.");
//                ss = (ServerSocketConnection) Connector.open("socket://:1735");
//                System.out.println(ss.getLocalAddress());
//                System.out.println("Opening socket connection.");
//                sock = ss.acceptAndOpen();
//                System.out.println("Opening input stream.");
//                in = sock.openDataInputStream();
//            }
//            catch(java.io.IOException er) {
//                System.out.println("Problem opening tcp connection");
//                System.out.println(er.getMessage());
//            }
//        }
//    }
//
//    public void getTCPData() {
//        if(TCP) {
//            //Set up Variables
//            String one,two,three,four,five = "";
//            String str = new String();
//            StringBuffer buf = new StringBuffer();
//            int i = 0;
//            int availablebytes = 0;
//
//            //Check for new bytes
//            try {
//                availablebytes = in.available();
//            }
//            catch (java.io.IOException fdkjg) {
//            }
//
//            //If enough new bytes for a message, process new bytes
//            if(availablebytes >= 45) {
//                try {
//                    in.skipBytes(in.available() - 45);
//                }
//                catch (java.io.IOException errr) {
//                    System.out.println("problem reading tcp connection");
//                }
//                try {
//                    for(i = 1; i <= 45; i++) {
//                        buf.append(in.readUnsignedByte() - 48);
//                    }
//                }
//                catch (java.io.IOException errrr) {
//                    System.out.println("problem reading tcp connection");
//                }
//
//                str = buf.toString();
//                int beginindex = str.indexOf('2');
//                
//                try{
//                    one = str.substring(beginindex+1,beginindex+9);
//                    two = str.substring(beginindex+10,beginindex+18);
//                    three = str.substring(beginindex+19,beginindex+27);
//                    four = str.substring(beginindex+28,beginindex+36);
//                    five = str.substring(beginindex+37,beginindex+45);
//                } catch (java.lang.Exception exd) {
//                    System.out.println("Exception Taking Substrings!!!!");
//                    return;
//                }
//
//                //Convert Five Binary Strings to Ints
//                try {
//                    rollerState = eightBitToInt(one);
//                    unusedState = eightBitToInt(two);
//                    armState = eightBitToInt(three);
//                    conveyorState = eightBitToInt(four);
//                    spare = eightBitToInt(five);
//                }
//                catch (java.lang.NumberFormatException err ) {
//                    System.out.println("problem converting to int");
//                }
//            }
//        }
//    }
    }
}