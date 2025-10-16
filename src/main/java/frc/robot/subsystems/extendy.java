// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class extendy extends SubsystemBase {
  /* Motor declaration */
  public TalonFX spoolMotor;
  public TalonFX spool2;
  public Encoder extendyPosition; 
  public double elevatorHeight;
  public double L4ExtensionTargetDistance;
  public double L3ExtensionTargetDistance;
  public double L2ExtensionTargetDistance;
  public double L1ExtensionTargetDistance;
  public double baseExtensionTargetDistance;
  public PIDController elevatorPID;
  public PIDController downPID;
  public ElevatorFeedforward elevatorFeedForward;
  private BooleanSupplier a,b,c,d,e,f;
  private static final double LEVEL_1_HEIGHT = 0;
  private static final double LEVEL_2_HEIGHT = 6.11;
  private static final double LEVEL_3_HEIGHT = 14.88;
  private static final double LEVEL_4_HEIGHT = 27.75;
  private static final double BASE_HEIGHT = 0;
  /** Creates a new extendy. */
  public extendy(BooleanSupplier a, BooleanSupplier b, BooleanSupplier c, BooleanSupplier  d, BooleanSupplier e, BooleanSupplier f) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.d = d;
    this.e = e;
    this.f = f;

    spoolMotor = new TalonFX(22);
    spool2 = new TalonFX(23);
    extendyPosition = new Encoder(4,3, false, Encoder.EncodingType.k2X);
    extendyPosition.setDistancePerPulse(-1.79*Math.PI/2048);
    elevatorPID = new PIDController(2.5, 0, 0);
    downPID = new PIDController(.25, 0, 0);
    elevatorFeedForward = new ElevatorFeedforward(0, 0.30, 37.92, 0.01);
    spoolMotor.setNeutralMode(NeutralModeValue.Brake);
  }
 
  public  double getElevatorHeight(){
    elevatorHeight = extendyPosition.getDistance();
    return elevatorHeight;
   //return 0.0;
  }
  public double determineTargetHeight() {
    return a.getAsBoolean() ? LEVEL_1_HEIGHT : 
           (b.getAsBoolean() ? LEVEL_2_HEIGHT : 
           (c.getAsBoolean() ? LEVEL_3_HEIGHT : 
           (d.getAsBoolean() ? LEVEL_4_HEIGHT : 
           (e.getAsBoolean() ? BASE_HEIGHT :
           (f.getAsBoolean() ? LEVEL_4_HEIGHT : 0)))));
}

  @Override
  public void periodic() {
 
    SmartDashboard.putNumber("elevator height", getElevatorHeight());//# of rotations div or multiply by a factor for actual height
    // This method will be called once per scheduler run
  }
}
