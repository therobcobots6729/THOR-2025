// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extendy;
import frc.robot.subsystems.flippy;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class processorExtension extends Command {
  private final extendy e_Extendy;
  private flippy f_Flippy;
  /** Creates a new processorExtension. */
  public processorExtension(extendy e_Extendy, flippy f_Flippy) {
    addRequirements(e_Extendy);
    this.e_Extendy = e_Extendy;
    this.f_Flippy  = f_Flippy;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    e_Extendy.spoolMotor.setVoltage(-e_Extendy.downPID.calculate(e_Extendy.extendyPosition.getDistance(), 10) + -e_Extendy.elevatorFeedForward.calculate(0,0));
    e_Extendy.spool2.setVoltage(e_Extendy.downPID.calculate(e_Extendy.extendyPosition.getDistance(), 10) + e_Extendy.elevatorFeedForward.calculate(0,0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    e_Extendy.spoolMotor.setVoltage(-e_Extendy.elevatorFeedForward.calculate(0,0));
    e_Extendy.spool2.setVoltage(e_Extendy.elevatorFeedForward.calculate(0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(e_Extendy.elevatorPID.calculate(e_Extendy.extendyPosition.getDistance(), 10)) < .005){
      return true;
    }
    else{
      return false;
    }
  }
}
