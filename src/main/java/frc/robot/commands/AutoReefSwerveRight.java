
    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;




import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import frc.robot.subsystems.ScoringLog;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.extendy;
import frc.robot.subsystems.limelight;




public class AutoReefSwerveRight extends Command {
  private Swerve s_Swerve;
  private ScoringLog sLog;
  private extendy e_Extendy;
  private limelight l_Limelight;
  double dAdj, xAdj, yawAdj;
  public AutoReefSwerveRight(Swerve s_Swerve, extendy e_Extendy, ScoringLog sLog, limelight l_Limelight) {
    this.s_Swerve = s_Swerve;
    this.e_Extendy = e_Extendy;
    this.l_Limelight = l_Limelight;
    this.sLog = sLog;
    addRequirements(s_Swerve, e_Extendy, sLog, l_Limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
        /* Get Values, Deadband */
   
    /* Drive */ // change 4 and 7 before a real match*/
        
          yawAdj = (l_Limelight.getTargetYaw() - s_Swerve.getPose().getRotation().getDegrees());
          xAdj = -(68.8-l_Limelight.x2);  
          dAdj = (68.6 - l_Limelight.y2);    
         // contains all reef IDs
        //(limeligt.ID == 1.0 || limelight.ID == 2.0 || limelight.ID == 12.0 || limelight.ID == 13.0)     // all source IDs
          s_Swerve.drive(
            new Translation2d(dAdj*.0003,xAdj*.0007)
            .times(Constants.Swerve.maxSpeed ), 
            yawAdj *.2, 
            false, 
            true);
         } 
        
      

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(
            new Translation2d(0,0),
            0, 
            false, 
            true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
        return false;
      }
    }
  
    

