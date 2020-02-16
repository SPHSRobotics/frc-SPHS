/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.opencv.core.MatOfPoint;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

//OKYERE EDIT: TIME IMPORTS
import java.util.Date;

import javax.lang.model.util.ElementScanner6;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Instanciante the motor controller objects

  public int position=0;
  public Boolean ArmSpeedReduce =true;
  public double speedMod = .7;
  public double armMod = 1;
  public Talon frontRightDrive=  new Talon(0);
  public Talon frontLeftDrive= new Talon(2);
  public Talon backRightDrive= new Talon(1);
  public Talon backLeftDrive= new Talon(3);
  public Talon armTest1= new Talon(4);
  public PowerDistributionPanel pdp = new PowerDistributionPanel();
  public DigitalInput limitSwitch =  new DigitalInput(1);
  public AnalogInput pot = new AnalogInput(0);
  public double potVal = 0;
  public double setPoint = 0;
  //public Solenoid front = new Solenoid(0);
  //public Solenoid front2 = new Solenoid(1);
  public DoubleSolenoid front = new DoubleSolenoid(0,1);

  public DoubleSolenoid back = new DoubleSolenoid(2,3);
  //public Solenoid back = new Solenoid(2);
  //public Solenoid back2 = new Solenoid(3);
  //public SpeedControllerGroup manuelArm = new SpeedControllerGroup(armTest1,armTest1);
  public SpeedControllerGroup rightMotors= new SpeedControllerGroup(frontRightDrive,backRightDrive);
  public SpeedControllerGroup leftMotors= new SpeedControllerGroup(frontLeftDrive,backLeftDrive);
  //SpeedControllerGroup leftMotorGroup= new SpeedControllerGroup(leftMaster, leftSlave);
  //SpeedControllerGroup rigtMotorGroup= new SpeedControllerGroup(rightMaster, rightSlave);

  //public Spark


  //instance differental drive object

  public DifferentialDrive drive= new DifferentialDrive(leftMotors, rightMotors);
  //public DifferentialDrive driveArm= new DifferentialDrive(manuelArm, manuelArm);
  //create a constructor function
  // negates right side

  public DriveSubsystem(){
   
  }

  //add a method for driving manual

  public void setSetPoint(){
    setPoint = pot.getVoltage();
  }

  /* code 0:

  , double aty, double ary

  */


  /* code 1:
  
  SmartDashboard.putNumber("attacklefty", aty);
  SmartDashboard.putNumber("attackrighty", ary);

  */
  Date date = new Date();
  public long getTimeNow(){
    date = new Date();
    return date.getTime();
  }
  public void AutoFunctions(boolean AutoMove, boolean button10, boolean button11){
    long timeNow;
    long timeStop;
    if(AutoMove){
      timeNow= getTimeNow();
      timeStop=timeNow+1000;
      while(timeNow<=timeStop){
        timeNow=getTimeNow();
        drive.tankDrive((0.99),(0.99));
      }
    }
    if(button10){    
      timeNow=getTimeNow();
      timeStop=timeNow+1000;
      while(timeNow<=timeStop){
        timeNow=getTimeNow();
        armTest1.setSpeed(-.4 *armMod);
        setPoint =4.4;
      }
    }
    if(button11){
      timeNow=getTimeNow();
      timeStop=timeNow+1000;
      while(timeNow<=timeStop){
        timeNow=getTimeNow();
        armTest1.setSpeed(.4 *armMod);
        setPoint = 4.6;
      }
    }
      speedMod=0.7;
      SmartDashboard.putNumber("Front Left Motor Speed",frontLeftDrive.getSpeed());
      SmartDashboard.putNumber("Back Left Motor Speed",backLeftDrive.getSpeed());
      SmartDashboard.putNumber("Front Right Motor Speed",frontRightDrive.getSpeed());
      SmartDashboard.putNumber("Back Right Motor Speed",backRightDrive.getSpeed());
      SmartDashboard.putNumber("PDP Temperature",pdp.getTemperature());
      SmartDashboard.putNumber("Front Left Motor Current",pdp.getCurrent(14));
      SmartDashboard.putNumber("Back Left Motor Current",pdp.getCurrent(15));
      SmartDashboard.putNumber("Front Right Motor Current",pdp.getCurrent(1));
      SmartDashboard.putNumber("Back Right Motor Current",pdp.getCurrent(0));
  
      potVal = pot.getVoltage();
      SmartDashboard.putNumber("Poteintiometer Voltage", potVal);
      SmartDashboard.putNumber("Arm Speed",armTest1.getSpeed());
      SmartDashboard.putNumber("Arm Current",pdp.getCurrent(2)); 
    }

  
  public void manualDrive(double left, double right, boolean buttonUR, boolean buttonBL,
   boolean buttonBR, int pov,boolean buttonA,double speedSetting,
   boolean buttonBack){//boolean names are the buttons they stand for
   
    /*if(joyY < 0)
    {
      joyY = joyY * 0.5;
    }*/
    drive.tankDrive((left * speedSetting*1.2), (right * speedSetting*1.2));//trying to make the controller less sensistive by reducing the "speed" by 10%
    SmartDashboard.putNumber("Front Left Motor Speed",frontLeftDrive.getSpeed());
    SmartDashboard.putNumber("Back Left Motor Speed",backLeftDrive.getSpeed());
    //SmartDashboard.putNumber("Left",left);
    //SmartDashboard.putNumber("Right",right);
    SmartDashboard.putNumber("Front Right Motor Speed",frontRightDrive.getSpeed());
    SmartDashboard.putNumber("Back Right Motor Speed",backRightDrive.getSpeed());
    
    SmartDashboard.putNumber("PDP Temperature",pdp.getTemperature());
    /*insert code 1*/
    SmartDashboard.putNumber("Front Left Motor Current",pdp.getCurrent(14));
    SmartDashboard.putNumber("Back Left Motor Current",pdp.getCurrent(15));
    SmartDashboard.putNumber("Front Right Motor Current",pdp.getCurrent(1));
    SmartDashboard.putNumber("Back Right Motor Current",pdp.getCurrent(0));

    potVal = pot.getVoltage();
    SmartDashboard.putNumber("Poteintiometer Voltage", potVal);
    
    if(buttonUR&&position<500000){
      armTest1.setSpeed(0.5);
      position++;
    }
    else if(buttonBR&&position>0){
      armTest1.setSpeed(-0.5);
      position--;
    }
    else if(buttonBL){
      while(position<200000){
        armTest1.setSpeed(0.5);
        position++;
      }
      while(position>200000){
        armTest1.setSpeed(-0.5);
        position--;
      }
    }else if(buttonA){
      while(position<350000){
        armTest1.setSpeed(0.5);
        position++;
      }
      while(position>350000){
        armTest1.setSpeed(-0.5);
        position--;
      }
    }else
      armTest1.setSpeed(0);     
      
      SmartDashboard.putNumber("Set Point",setPoint);

      if((Math.abs(setPoint- potVal)) < 0.01){
        //armTest1.setSpeed(0.0);
      }else if(potVal < setPoint){
       // armTest1.setSpeed(-.1);
      }else if(potVal > setPoint){
       // armTest1.setSpeed(-.1);
      }

/*if(joy6){
//  gug.setClosedLoopControl(true);  
 // front.set(true);
  //front2.set(true);
  front.set(DoubleSolenoid.Value.kReverse);
  

}else{
  //front.set(false);
 // front2.set(false);
 front.set(DoubleSolenoid.Value.kForward);
 
//  gug.setClosedLoopControl(false);
}

if(joy7){
  //back.set(true);
  //back2.set(true);
  back.set(DoubleSolenoid.Value.kForward);
}else{
  //back.set(false);
  //back2.set(false);
  back.set(DoubleSolenoid.Value.kReverse);
}*/

      SmartDashboard.putNumber("Arm Speed",armTest1.getSpeed());
      SmartDashboard.putNumber("Arm Current",pdp.getCurrent(2));}     //pdp - powerDistributionPanel
      
      /*if(a && potVal < 4.8){
      armTest1.setSpeed(1);
      }else if(b && !limitSwitch.get()){
      armTest1.setSpeed(-1);
      }else{
      armTest1.setSpeed(0.0);
      }*/
    
       
    
  //}



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   setDefaultCommand(new DriveManuallyCommand());
  }
}
