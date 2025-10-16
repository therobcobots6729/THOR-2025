// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sucky;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runSucky extends Command {
  public BooleanSupplier a;
  public BooleanSupplier b;
  /** Creates a new runSucky. */
  public runSucky(sucky s_Sucky, BooleanSupplier a, BooleanSupplier b) {
    addRequirements(s_Sucky);
    this.a = a; 
    this.b = b;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (a.getAsBoolean()){
      sucky.leftMotor.set(-1);
      sucky.rightMotor.set(-1);
    }
    else if (b.getAsBoolean()){
      sucky.leftMotor.set(1);
      sucky.rightMotor.set(1);
    }
    else{
      sucky.leftMotor.set(0);
      sucky.rightMotor.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
