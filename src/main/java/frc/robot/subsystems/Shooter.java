// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




//TODO test this on the robot

package frc.robot.subsystems;
import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private SparkMaxConfig motorConfig = new SparkMaxConfig();
  private SparkMax leftMotor = new SparkMax(Constants.ShooterConstants.leftMotorID,MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(Constants.ShooterConstants.rightMotorID, MotorType.kBrushless);
  private DigitalInput coralSensor2 = new DigitalInput(Constants.ShooterConstants.coralSensor2Front);
 // private DigitalInput coralSensor1 = new DigitalInput(Constants.ShooterConstants.coralSensor1Back);

  public boolean hasCoral = true;

  BooleanPublisher backCoralSensorPublisher;

@AutoLogOutput

  //private boolean coralSensor1Output;
  private boolean coralSensor2Output;

  //private Boolean coralSensor3BooleanSupplier;

  public Shooter() {
    motorConfig
    .idleMode(IdleMode.kBrake)
    .inverted(true);


    rightMotor.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    leftMotor.configure(motorConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

  }

  public void spinShooters(double rightMotorSpeed, double leftMotorSpeed){
    leftMotor.set(leftMotorSpeed);
    rightMotor.set(rightMotorSpeed);
  }
  //public boolean getCoralSensor1(){
  //  return coralSensor1.get();
 // }
  public boolean getCoralSensor2(){
    return coralSensor2.get();
  }

  public void stopShooter(){
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void periodic() {

  //  coralSensor1Output =getCoralSensor1();
    coralSensor2Output =getCoralSensor2();
    
 //   backCoralSensorPublisher.setDefault(false);
 //   backCoralSensorPublisher.set(coralSensor3Output);
  }
}