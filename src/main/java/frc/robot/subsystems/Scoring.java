package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DutycycleEncoder;

public class Scoring extends SubsystemBase {
    boolean scoringEnabled = false;
    //macro positions
    double[] armPos = {2.55, 2.85, 2.7};
    double[] wristPos = {1.2, 1.85, 2.25, 2.4, 2.5};
    double[] elevatorPos = {0, 4300, 10762, 17000};
    //targets
    double aTarget = armPos[0];
    double eTarget = elevatorPos[0];
    double wTarget = wristPos[0];
    //motors
    Spark Wrist = new Spark(9);
    Spark Skullcrusher = new Spark(7);
    Spark Piranha = new Spark(8);
    //P values
    double kPWrist = 1;
    double kPArm = 2.8;
    double kPElevator = .0005;
    //relative encoders
    private final Encoder elevatorEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k2X);//normally 4,5
    //absolute encoder
    private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(5, 4.0, 0.0);
    private final DutyCycleEncoder skullcrushEncoder = new DutyCycleEncoder(9,4.0,0.0);

    public void L4Raise() {
        wTarget = wristPos[4];
        aTarget = armPos[0];
        eTarget = elevatorPos[3];
    }

    public void StopAll() {
        //safety measure if auton is finished
        Wrist.stopMotor();
        Piranha.stopMotor();
        elevatorMotor.stopMotor();
        Skullcrusher.set(.15);
    }
    

    public void Activated(boolean isEnabled) {
        scoringEnabled = isEnabled;
        SmartDashboard.putBoolean("Auton Status:", scoringEnabled);
    }

    public void Loading() {
        wTarget = wristPos[1];
        aTarget = armPos[0];
        eTarget = elevatorPos[0];
        if (scoringEnabled) {
                   Piranha.set(.50); 
        }
    }
    public void releaseCoral() {
        Piranha.set(-.5);
    }

    @Override
    public void periodic() {

        if (scoringEnabled) { //checks if auton is enabled
            //only do calculations if auton is enabled
                double currWristPos = wristEncoder.get();
                double currArmPos = skullcrushEncoder.get();
                double currElevPos = elevatorEncoder.getDistance();
                double wristError = Math.abs(currWristPos - wTarget);
                double armError = Math.abs(currArmPos - aTarget);
                double elevError = Math.abs(currElevPos - eTarget);
                    if(wTarget - .02 > currWristPos){
                        Wrist.set(wristError * kPWrist);
                }   else if (wTarget + .02 < currWristPos){
                        Wrist.set(-wristError * kPWrist);
                }   else {
                        Wrist.stopMotor();
                }
                    if(eTarget - 25 > currElevPos){
                        elevatorMotor.set(-elevError * kPElevator); //up
                }   else if(eTarget + 25 < currElevPos){
                        elevatorMotor.set(elevError * kPElevator); //down
                }   else{
                    elevatorMotor.stopMotor(); 
                }

                    if(aTarget - .05 > currArmPos){
                        Skullcrusher.set(-kPArm * armError);
                } else if(aTarget + .05 < currArmPos){
                    Skullcrusher.set(kPArm * armError);
                } else {
                    Skullcrusher.set(.15);
        }  
    }
}
