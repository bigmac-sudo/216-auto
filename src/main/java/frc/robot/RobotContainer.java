// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  double[] armPos = {2.55, 2.85, 2.7};
  double[] wristPos = {1.2, 1.85, 2.25, 2.4, 2.5};
  double[] elevatorPos = {0, 4300, 10762, 17000};
  double[] algaePos = {2.4, 3.2};
  double wTarget = wristPos[0];
  double aTarget = armPos[0];
  double eTarget = elevatorPos[0];
  double alTarget = algaePos[0]; 
  double kPWrist = 1;
  double kPArm = 2.8;
  double kPElevator = .0005;
  double kPAlgae = .15;
  Spark Piranha = new Spark(8);
  Spark Wrist = new Spark(9);
  Spark Skullcrusher = new Spark(7);
      double currWristPos = wristEncoder.get();
    double currArmPos = skullcrushEncoder.get();
    double currElevPos = elevatorEncoder.getDistance();
    double wristError = Math.abs(currWristPos - wTarget);
    double armError = Math.abs(currArmPos - aTarget);
    double elevError = Math.abs(currElevPos - eTarget);
    wTarget = wristPos[4];
    aTarget = armPos[0];
    eTarget = elevatorPos[3];
    private final Encoder elevatorEncoder = new Encoder(3, 4, false, Encoder.EncodingType.k2X);//normally 4,5
    //absolute encoder
    private final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(5, 4.0, 0.0);
    private final DutyCycleEncoder skullcrushEncoder = new DutyCycleEncoder(9,4.0,0.0);


    NamedCommands.registerCommand("L4", Commands.runOnce(() ->{
        
    if(wTarget - .02 > currWristPos){
      Wrist.set(wristError * kPWrist);
    } else if (wTarget + .02 < currWristPos){
      Wrist.set(-wristError * kPWrist);
    } else {
      Wrist.stopMotor();
    }

    if(eTarget - 25 > currElevPos){
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
}));

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller joystick = new CommandPS4Controller(1);
    // private final CommandPS4Controller operator = new CommandPS4Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.R3().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.L3().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.share().and(joystick.triangle()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.share().and(joystick.square()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.options().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.options().and(joystick.square()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.L1().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        return new SequentialCommandGroup(
            drivetrain.applyRequest(()->
            //drive.withVelocityX(.25*MaxSpeed) //blue
            drive.withVelocityX(-.25*MaxSpeed) //red
            .withVelocityY(0)
            .withRotationalRate(0)).withTimeout(1.5),
            drivetrain.applyRequest(()->
            drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)).withTimeout(10));
    }
}