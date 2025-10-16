package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelight extends SubsystemBase { // Capitalized class name (follows Java conventions)
  
  public static double x1;
public double y1;
public static double reefd;
public double processord;
  public static double ID1;
  public static double stationd;
  public static double x2;
public double y2;
public double ID2;

  private final NetworkTable table1 = NetworkTableInstance.getDefault().getTable("limelight1"); // Forward limelight
  private final NetworkTableEntry tx1 = table1.getEntry("tx");
  private final NetworkTableEntry ty1 = table1.getEntry("ty");
  private final NetworkTableEntry tid1 = table1.getEntry("tid");

  private final NetworkTable table2 = NetworkTableInstance.getDefault().getTable("limelight2"); // Backward limelight
  private final NetworkTableEntry tx2 = table2.getEntry("tx");
  private final NetworkTableEntry ty2 = table2.getEntry("ty");
  private final NetworkTableEntry tid2 = table2.getEntry("tid");

  private final double limelight1MountAngleDegrees = 0; // Change as needed
  private final double limelight1LensHeightInches = 18; // Change as needed
  private final double limelight2MountAngleDegrees = 0; // Change as needed
  private final double limelight2LensHeightInches = 24; // Avoid zero to prevent division errors

  /** Constructor */
  public limelight() {}

  /** Gets the detected AprilTag ID from the forward Limelight */
  public  double getTagID() {
      return tid1.getDouble(0.0);
  }

  /** This method runs periodically */
  @Override
  public void periodic() {
      // Update values every cycle
      double targetOffsetAngle_Vertical1 = ty1.getDouble(0.0);
      double targetOffsetAngle_Vertical2 = ty2.getDouble(0.0);

      double reefHeightInches = 12.125;
      double angleToReefDegrees = limelight1MountAngleDegrees + targetOffsetAngle_Vertical1;
      double angleToReefRadians = Math.toRadians(angleToReefDegrees);
      reefd = (reefHeightInches - limelight1LensHeightInches) / Math.tan(angleToReefRadians);

      double stationHeightInches = 58.5;
      double angleToStationDegrees = limelight2MountAngleDegrees + targetOffsetAngle_Vertical2;
      double angleToStationRadians = Math.toRadians(angleToStationDegrees);
      stationd = (stationHeightInches - limelight2LensHeightInches) / Math.tan(angleToStationRadians);

      double processorHeightInches = 51.125;
      double angleToProcessorDegrees = limelight1MountAngleDegrees + targetOffsetAngle_Vertical1;
      double angleToProcessorRadians = Math.toRadians(angleToProcessorDegrees);
      processord = (processorHeightInches - limelight1LensHeightInches) / Math.tan(angleToProcessorRadians);

      // Read values periodically
      x1 = tx1.getDouble(0.0);
      y1 = ty1.getDouble(0.0);
      ID1 = tid1.getDouble(0.0);
      x2 = tx2.getDouble(0.0);
      y2 = ty2.getDouble(0.0);
      ID2 = tid2.getDouble(0.0);
      SmartDashboard.putNumber("reefx", x1);
      //SmartDashboard.putNumber("reefx", x2);
    
  }
}
