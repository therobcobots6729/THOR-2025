// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sucky extends SubsystemBase {
  public static SparkMax leftMotor;
  public static SparkMax rightMotor;
  public static SparkMax intakeMotor;
  private SparkMaxConfig suckyMotors;
  public static DigitalInput beam;
  /** Creates a new sucky. */
  public sucky() {
    suckyMotors = new SparkMaxConfig();
    leftMotor = new SparkMax(15, MotorType.kBrushless);
    rightMotor = new SparkMax(25 , MotorType.kBrushless);
    //intakeMotor = new SparkMax(0, MotorType.kBrushless);
    configSuckyMotors();
    beam = new DigitalInput(2);
    
   
  }
 private void configSuckyMotors()
    {

        suckyMotors
            .inverted(false)
            .idleMode(IdleMode.kBrake);
            
            
    
        leftMotor.configure(suckyMotors, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  
        rightMotor.configure(suckyMotors, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);     
       
    }
    public BooleanSupplier finish(){

      return ()-> beam.get();
    }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("beams", beam.get());
    
    // This method will be called once per scheduler run
  }
}
