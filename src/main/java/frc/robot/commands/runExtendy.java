// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extendy;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runExtendy extends Command {
  public BooleanSupplier a;
  public BooleanSupplier b;
  private final extendy e_Extendy;
  /** Creates a new runExtendy. */
  public runExtendy(extendy e_Extendy, BooleanSupplier a, BooleanSupplier b) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(e_Extendy);
    this.e_Extendy =  e_Extendy;
    this.a = a;
    this.b = b;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = a.getAsBoolean() ? -.2 : (b.getAsBoolean() ? .2 : 0);
      e_Extendy.spoolMotor.setControl(new DutyCycleOut(power));
      e_Extendy.spool2.setControl(new DutyCycleOut(-power));

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double holdVoltage = e_Extendy.elevatorFeedForward.calculate(0, 0);
    e_Extendy.spoolMotor.setControl(new VoltageOut(-holdVoltage));
    e_Extendy.spool2.setControl(new VoltageOut(holdVoltage));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return !a.getAsBoolean() && !b.getAsBoolean();
  }
}
