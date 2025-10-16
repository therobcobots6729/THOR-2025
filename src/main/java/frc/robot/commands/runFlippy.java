// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flippy;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runFlippy extends Command {
  public BooleanSupplier a;
  public BooleanSupplier b;
  private final flippy f_Flippy;
  /** Creates a new runFlippy. */
  public runFlippy(flippy f_Flippy, BooleanSupplier a, BooleanSupplier b) {
    addRequirements(f_Flippy);
    this.f_Flippy = f_Flippy;
    this.a = a;
    this.b =b;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = a.getAsBoolean() ? -0.25 : (b.getAsBoolean() ? 0.25 : 0);
    f_Flippy.leftPivot.setControl(new DutyCycleOut(power));
    f_Flippy.rightPivot.setControl(new DutyCycleOut(power));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double holdVoltage = f_Flippy.wristFeedForward.calculate(3.14, 0);
    f_Flippy.leftPivot.setControl(new VoltageOut(holdVoltage));
    f_Flippy.rightPivot.setControl(new VoltageOut(holdVoltage));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !a.getAsBoolean() && !b.getAsBoolean();
  }
}
