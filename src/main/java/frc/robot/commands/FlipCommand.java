package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flippy;
import com.ctre.phoenix6.controls.VoltageOut;

public class FlipCommand extends Command {
  private final flippy f_Flippy;
 
  

  /**
   * Creates a new FlipCommand.
   * @param f_Flippy The wrist subsystem.
   *
   */
  public FlipCommand(flippy f_Flippy) {
    this.f_Flippy = f_Flippy;
  
    addRequirements(f_Flippy);
  }

  @Override
  public void execute() {
    double pidOutput = f_Flippy.wristPID.calculate(f_Flippy.WristPosition(), f_Flippy.determineTargeAngle());
    double feedForward = f_Flippy.wristFeedForward.calculate(f_Flippy.WristPosition(), 0);
    double voltage = pidOutput + feedForward;

    f_Flippy.leftPivot.setControl(new VoltageOut(voltage));
    f_Flippy.rightPivot.setControl(new VoltageOut(voltage));
  }

  @Override
  public void end(boolean interrupted) {
    double holdVoltage = f_Flippy.wristFeedForward.calculate(f_Flippy.WristPosition(), 0);
    f_Flippy.leftPivot.setControl(new VoltageOut(holdVoltage));
    f_Flippy.rightPivot.setControl(new VoltageOut(holdVoltage));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(f_Flippy.wristPID.calculate(f_Flippy.wristEncoder.get(), f_Flippy.determineTargeAngle())) < 0.005;
  }
}
