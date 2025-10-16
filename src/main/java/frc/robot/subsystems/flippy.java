// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class flippy extends SubsystemBase {  
  /** Creates a new flippy. */
  public TalonFX leftPivot;
  public TalonFX rightPivot;
  public DutyCycleEncoder wristEncoder;
  public double wristAngle;
  public double backWristTargetDistance;
  public double upWristTargetDistance;
  public double downWristTargetDistance;
  public double forwardWristTargetDistance;
  public PIDController wristPID;
  public ArmFeedforward wristFeedForward;
  private BooleanSupplier a,b,c,d,e,f;
  private double OUT = 180;
  private double IN = 0;
  private double UP = 235;

  public flippy(BooleanSupplier a, BooleanSupplier b, BooleanSupplier c, BooleanSupplier  d, BooleanSupplier e, BooleanSupplier f) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.d = d;
    this.e = e;
    this.f = f;
    leftPivot = new TalonFX(18);
    rightPivot = new TalonFX(19);
    leftPivot.setInverted(true);
    rightPivot.setInverted(true);
    wristEncoder = new DutyCycleEncoder(0);
    leftPivot.setNeutralMode(NeutralModeValue.Brake);
    rightPivot.setNeutralMode(NeutralModeValue.Brake);
    wristFeedForward = new ArmFeedforward(0,.32, .45, .01);
    wristPID = new PIDController(2.75, 0, 0);
  }
  
  public double determineTargeAngle() {
    return a.getAsBoolean() ? OUT : 
           (b.getAsBoolean() ? OUT : 
           (c.getAsBoolean() ? OUT : 
           (d.getAsBoolean() ? OUT : 
           (e.getAsBoolean() ? IN :
           (f.getAsBoolean() ? UP : 0)))));

  }
  public double WristPosition(){
    wristAngle = (wristEncoder.get() * 360) - 281;
    return Math.toRadians(wristAngle);
  }
  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("wrist angle", wristAngle);// 0 is a placeholder for an offset
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("error",Math.abs(Math.toRadians(10)-WristPosition()));
    SmartDashboard.putBoolean("connection", wristEncoder.isConnected());
  }
}
