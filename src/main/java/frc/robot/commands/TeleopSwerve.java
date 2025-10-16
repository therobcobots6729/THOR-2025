package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.limelight;
import frc.robot.subsystems.extendy;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private extendy e_Extendy;
  private double speed;
  private BooleanSupplier robotCentricSup;
  

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      extendy e_Extendy,
      BooleanSupplier robotCentricSup
      ) {
    this.s_Swerve = s_Swerve;
    this.e_Extendy = e_Extendy;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    if (e_Extendy.elevatorHeight>6){
      speed = .25;
    }
    else{
      speed = 1;
    }
    double translationVal =
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    
    /* Drive */
        /*  if ((limelight.ID == 4.0 || limelight.ID == 7.0) && Math.abs(translationVal) < .1 && Math.abs(strafeVal) < .1 && Math.abs(rotationVal) < .1 && Math.abs(limelight.x) > 2) {
          s_Swerve.drive(
            new Translation2d(0,0), limelight.x *-.02, true, false);
         }                      
         else{*/
      s_Swerve.drive(
        new Translation2d(translationVal, strafeVal)
            .times(Constants.Swerve.maxSpeed *speed ),
        rotationVal * Constants.Swerve.maxAngularVelocity * speed,
        false,
       // !robotCentricSup.getAsBoolean(),
        true);
    }}

