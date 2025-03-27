// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license f
package frc.robot;
import javax.lang.model.util.ElementScanner14;

import org.opencv.core.Mat.Atable;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.controller.PIDController;



public class Robot extends TimedRobot {
  boolean starttimer = false;
  boolean auto_selected;
  int wristlevel = 0;
  Timer timer = new Timer();
  //constants for arm/wrsit
  double[] armPos = {2.55, 3, 2.20, 2.4, 2.8};
  double[] wristPos = {1.05, 1.95, 2.2, 2.25, 1.6,2.6};
  double[] elevatorPos = {-10, 1200, 7900, 18350, 1500, 8300};
  double[] algaePos = {2.40, 2.8, 2.55,3};
  double wTarget = wristPos[0];
  double aTarget = armPos[4];
  double eTarget = elevatorPos[0];
  double alTarget = algaePos[0]; 
  double kPWrist = .75;
  double kPArm = 3;
  double kPElevator = .0005;
  double kPAlgae = .3;
  UsbCamera camera1;
  NetworkTableEntry cameraSelection;
  private final Joystick operator = new Joystick(0);
  private final Joystick driver = new Joystick(1);
  Spark Piranha = new Spark(8);
  Spark Wrist = new Spark(9);
  Spark Skullcrusher = new Spark(7);
  Spark Algae = new Spark(6);
  TalonFX Torquer = new TalonFX(13);
  TalonFX AlgaeArm = new TalonFX(14);
  SparkMax elevatorMotor;
  //encoders
    private final Encoder elevatorEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k2X);//normally 4,5
    //absolute encoder
    private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(5, 4.0, 3.0);
    private final DutyCycleEncoder skullcrushEncoder = new DutyCycleEncoder(9,4.0,0.0);
    private final DutyCycleEncoder algaeEncoder = new DutyCycleEncoder(1, 4.0, 1.0);
    //auton selector
    private static final String kDefaultAuto = "Red Auto";
    private static final String kCustomAuto = "Blue Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Command m_autonomousCommand;
  // private final Joystick operator = new Joystick(0);
  // private final Joystick driver = new Joystick(1);
  // private final Drivetrain m_swerve = new Drivetrain();
  private final RobotContainer m_robotContainer;

  public Robot() {
    //elevatorEncoder.reset();
    m_robotContainer = new RobotContainer();
    elevatorMotor = new SparkMax(15,MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    elevatorMotor.configure(globalConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    CameraServer.startAutomaticCapture();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    //SmartDashboard.putNumber("Elevator", wristEncoder.get());
    SmartDashboard.putNumber("Elevator Encoder", elevatorEncoder.getDistance());
    SmartDashboard.putNumber("Wrist Encoder", wristEncoder.get());
    SmartDashboard.putNumber("ALgae Encoder", algaeEncoder.get());
    SmartDashboard.putNumber("Arm Encoder", skullcrushEncoder.get());

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    m_autoSelected =m_chooser.getSelected();
    switch (m_autoSelected) {
      case kCustomAuto:
        m_autonomousCommand = m_robotContainer.blueCommand();
        m_autonomousCommand.schedule();
        break;
      case kDefaultAuto:
      default:
        m_autonomousCommand = m_robotContainer.redCommand();
        m_autonomousCommand.schedule();
        break;
    }


  }

  @Override
  public void autonomousPeriodic() {
          switch (m_autoSelected) {
      case kCustomAuto:
      //code goes here
        break;
      case kDefaultAuto:
      default:
      //code goes here
        break;
    }
  
    // double currWristPos = wristEncoder.get();
    // double wristError = Math.abs(currWristPos - wTarget);

    // if(timer.get() > 2 && timer.get() < 9){
    //   wTarget = wristPos[4];
    // } else {
    //   wTarget = wristPos[0];
    // }

    // if(timer.get() > 4){
    //   Piranha.set(.3);
    // } else {
    //   Piranha.set(0);
    // }

    // // if(alTarget - .1 > currAlgaePos){
    // //   AlgaeArm.set(kPAlgae * algaeError);
    // // } else if(alTarget + .1 < currAlgaePos){
    // //   AlgaeArm.set(-kPAlgae * algaeError);
    // // } else {
    // //   AlgaeArm.stopMotor();
    // // }

    // if(wTarget - .05 > currWristPos){
    //   Wrist.set(wristError * kPWrist / 2);
    // } else if (wTarget + .05 < currWristPos){
    //   Wrist.set(-wristError * kPWrist / 2);
    // } else {
    //   Wrist.stopMotor();
    // }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel(); 
    } 

   }

  @Override
  public void teleopPeriodic() {
    // double currArmPos = skullcrushEncoder.get();
    // double currWirstPos = wristEncoder.get();
    double currAlgaePos = algaeEncoder.get();
    double currWristPos = wristEncoder.get();
    double currArmPos = skullcrushEncoder.get();
    double currElevPos = elevatorEncoder.getDistance();
    double wristError = Math.abs(currWristPos - wTarget);
    double armError = Math.abs(currArmPos - aTarget);
    double elevError = Math.abs(currElevPos - eTarget);
    double algaeError = Math.abs(currAlgaePos - alTarget);

    // System.out.println(wristEncoder.isConnected());
    // System.out.println(armError);
    if (operator.getPOV() == 0){
      eTarget = elevatorPos[5];
      aTarget = armPos[4];
      wTarget = wristPos[4];
    }
    //torquer
    if (driver.getRawButton(PS4Controller.Button.kCircle.value)){
      Torquer.set(.25);
    }else if (driver.getRawButton(PS4Controller.Button.kSquare.value)){
      Torquer.set(-.25);
    }else {
      Torquer.stopMotor();
    }
//algae buttons
    if (driver.getRawButton(PS4Controller.Button.kR1.value)) {
      Algae.set(1);
    } else if (driver.getRawButton(PS4Controller.Button.kR2.value)) {
      Algae.set(-1);
    }else {
      Algae.stopMotor();
    }
    // if (driver.getPOV() == 0) {
    //   Algae.set(1);
    // } else if (driver.getPOV() == 180) {
    //   Algae.set(-1);
    // }else {
    //   Algae.stopMotor();
    // }
    // if (operator.getRawButton(PS4Controller.Button.kL1.value)) {
    //   AlgaeArm.set(.25);
    // }else if (operator.getRawButton(PS4Controller.Button.kL2.value)) {
    //   AlgaeArm.set(-.25); 
    // }else {
    //   AlgaeArm.set(-.01);
    // }
    if (operator.getRawButton(PS4Controller.Button.kR1.value)) {
      alTarget = algaePos[0];
    }else if (operator.getRawButton(PS4Controller.Button.kR2.value)) {
      alTarget = algaePos[1];
    }
    
//coral buttons
    if (driver.getRawButton(PS4Controller.Button.kTriangle.value)) {
      Piranha.set(.75);//outtake
    }else if (driver.getRawButton(PS4Controller.Button.kL2.value)) {
      Piranha.set(-.55);//intake
    }else {
      Piranha.stopMotor();
    }


    //macros
    if (operator.getRawButton(PS4Controller.Button.kCross.value)){
      //loading
      wTarget = wristPos[0];
      aTarget = armPos[4];
      eTarget = elevatorPos[0];
    } else if (operator.getRawButton(PS4Controller.Button.kOptions.value)){
      //hang
      wTarget = wristPos[5];
      aTarget = armPos[4];
      eTarget = elevatorPos[0];
      alTarget = algaePos[3];
    } else if (operator.getRawButton(PS4Controller.Button.kTriangle.value)){
      //L2
      wTarget = wristPos[1];
      aTarget =  armPos[0];
      eTarget = elevatorPos[1];
    } else if (operator.getRawButton(PS4Controller.Button.kSquare.value)){
      //L1
      wTarget = wristPos[1];
      aTarget = armPos[4];
      eTarget = elevatorPos[0];
    } else if (operator.getRawButton(PS4Controller.Button.kCircle.value)){
      //l3
      wTarget = wristPos[1];
      aTarget = armPos[0];
      eTarget = elevatorPos[2];
    } //else if (operator.getRawButton(PS4Controller.Button.kOptions.value)){
    //   //algae
    //   wTarget = wristPos[1];
    //   aTarget = armPos[1];
    //   eTarget = elevatorPos[2];
    // } 
    else if (operator.getRawButton(PS4Controller.Button.kL2.value)){
      //L4
      wTarget = wristPos[2];
      aTarget = armPos[3];
      eTarget = elevatorPos[3];
    }

    

    if(wTarget - .02 > currWristPos){
      Wrist.set(wristError * kPWrist);
    } else if (wTarget + .02 < currWristPos){
      Wrist.set(-wristError * kPWrist);
    } else {
      Wrist.stopMotor();
    }

    if (driver.getRawButton(PS4Controller.Button.kCross.value)){
      elevatorMotor.set(-.3);
    } else if(eTarget - 25 > currElevPos){
      elevatorMotor.set(-elevError * kPElevator); //up
    } else if(eTarget + 25 < currElevPos){
      elevatorMotor.set(elevError * kPElevator); //down
    } else{
      elevatorMotor.stopMotor(); 
    }

    if(aTarget - .05 > currArmPos){
      Skullcrusher.set(-kPArm * armError);
    } else if(aTarget + .05 < currArmPos){
      Skullcrusher.set(kPArm * armError);
    } else {
      Skullcrusher.set(.15);
    }

    if(alTarget - .1 > currAlgaePos){
      AlgaeArm.set(kPAlgae * algaeError);
    } else if(alTarget + .1 < currAlgaePos){
      AlgaeArm.set(-kPAlgae * algaeError);
    } else {
      AlgaeArm.stopMotor();
    }



  

    // if (wTarget - .1 > currWirstPos){
    //   Wrist.set(.3);
    // } else if(wTarget + .1 < currWirstPos){
    //   Wrist.set(-.3);
    // } else {
    //   Wrist.stopMotor();
    // }
      // System.out.println("Wrist encoder: " + wristEncoder.get());

    // if(aTarget - .02 < currArmPos){
    //   Skullcrusher.set(.5);
    // } else if(aTarget + .02 > currArmPos){
    //   Skullcrusher.set(-.5);
    // } else {
    //   Skullcrusher.stopMotor();
    // }

//wrist buttons
    // if (operator.getRawButton(PS4Controller.Button.kSquare.value)) {
    //   Wrist.set(.35);
    //   //   if (wristEncoder.get() < 0.1 ) {
    //   //     Wrist.set(.3);
    //   // } else if (wristEncoder.get() >= 0.1) {
    //   //     Wrist.set(.2);
    //   // } else if (wristEncoder.get() > 0.3 && wristEncoder.get() < 0.5) {
    //   //     Wrist.set(0.05);
    //   // }
    // }else if (operator.getRawButton(PS4Controller.Button.kCircle.value)) {
    //   Wrist.set(-.35);
    // }else {
    //   Wrist.set(.05);
    // }

    //wrist encoder with PID
  //   if (operator.getRawButton(PS4Controller.Button.kSquare.value)) {
  //     Wrist.set(pid.calculate(wristEncoder.get(), 0.15));
  // }
    
   // arm pos 1
    // if (operator.getRawButton(PS4Controller.Button.kTriangle.value)){
    //   if (  < .1 ) {
    //     Wrist.set(.85);
    // } else if (offsetValueEncoderWrist > .13) {
    //     Wrist.set(.2);
    // } else if (offsetValueEncoderWrist > .135 && offsetValueEncoderWrist < .145) {
    //     Wrist.set(0.05);
    // }
    //   if (elevatorEncoder.get() < 1000){
    //     elevatorMotor.set(-.5); //elevator is opposite for whatever reason
    //   }else if (elevatoreEncoder.get() > 1200){
    //     elevatorMotor.set(-.25);
    //   } else if (elevatoreEncoder.get() >1000 && elevatoreEncoder.get() < 1400){
    //     elevatorMotor.set(.05);
    //   }
    //   if (skullcrushEncoder.get() < .3){
    //     Skullcrusher.set(.75);
    //   }else if (skullcrushEncoder.get() > .45){
    //     Skullcrusher.set(.4);
    //   } else if (skullcrushEncoder.get() > .6){
    //     Skullcrusher.set(.18);
    //   }
    // }
//elevator buttons
        
// if (operator.getRawButton(PS4Controller.Button.kL1.value)) {
//   elevatorMotor.set(.50);
// } else if (operator.getRawButton(PS4Controller.Button.kL2.value)) {
//     elevatorMotor.set(-.50);
// } else{
//   elevatorMotor.stopMotor();
// }
//     if (operator.getRawButton(PS4Controller.Button.kL1.value)) {
//         if (elevatorEncoder.get() < 4 ) {
//           elevatorMotor.set(-.7);
//       } else if (elevatorEncoder.get() >= 7) {
//           elevatorMotor.set(-.4);
//       } else if (elevatorEncoder.get() > 0.3 && elevatorEncoder.get() < 0.5) {
//           elevatorMotor.set(-0.01);
//       }
//     }else if (operator.getRawButton(PS4Controller.Button.kCircle.value)) {
//       Wrist.set(-.35);
//     }else {
//       Wrist.set(.05);
//     }

//elevator with PID:
  //   if (operator.getRawButton(PS4Controller.Button.kL1.value)) {
  //     elevatorEncoder.set(pid.calculate(elevatorEncoder.get(), 100));
  // }
// if (operator.getRawButton(PS4Controller.Button.kTriangle.value)) {
//   if (elevator_encoder.get() < 19000 ) {
//     elevatorMotor.set(.85);
// } else if (elevator_encoder.get() > 19001 && elevator_encoder.get() < 19500) {
//     elevatorMotor.set(.5);
// } else if (elevator_encoder.get() > 20080) {
//     elevatorMotor.set(0.0);
// }
// }

//skull crusher buttons
    // if (operator.getRawButton(PS4Controller.Button.kR2.value)) {
      
    //   Skullcrusher.set(.55);
    // }else if (operator.getRawButton(PS4Controller.Button.kR1.value)) {
    //   Skullcrusher.set(-.55);
    // }else {
    //   Skullcrusher.set(0.18);;
    // }
    //   //skullcrusher encoder
    //   if (operator.getRawButton(PS4Controller.Button.kR2.value)) {
    //     if (skullcrushEncoder.get() < 0.1 ) {
    //       Skullcrusher.set(.85);
    //   } else if (skullcrushEncoder.get() > 0.2 && skullcrushEncoder.get() < 0.3) {
    //       Wrist.set(.5);
    //   } else if (skullcrushEncoder.get() > 0.5) {
    //       Wrist.set(0.0);
    //   }
    // }

    //skullcrusher encoder with PID
  //   if (operator.getRawButton(PS4Controller.Button.kR2.value)) {
  //     Skullcrusher.set(pid.calculate(skullcrushEncoder.get(), 0.15));
  // }
    
  }
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

}
