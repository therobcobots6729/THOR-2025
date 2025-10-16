package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Capitalized class name (follows Java conventions)
  
 

public class limelight extends SubsystemBase {
  private Swerve s_Swerve;
 private PhotonCamera reefC;
 public PhotonTrackedTarget results;
  public int tid1;
  public double tid2;
  public double reefd, stationd, processord;
  public double x1, y1, ID1, x2, y2, ID2;

  /** Constructor */
  public limelight(Swerve s_Swerve) {
    reefC = new PhotonCamera("reef");
    this.s_Swerve=  s_Swerve;
   
  }

 
  /** This method runs periodically */

  @Override
public void periodic() {
    // Fetch new botpose values every cycle
    var result = reefC.getLatestResult();  // Get latest PhotonVision result
    var results = result.getBestTarget();  // Get best detected target
    
    if (result.hasTargets() && results != null) {  // Ensure a target is detected
        tid1 = results.getFiducialId();  // Get the AprilTag ID
    } else {
        tid1 = 0;  // Default value if no target is found
    }
  
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("reef");
  x1 =  table.getEntry("targetPixelsX").getDouble(0);
  y1 =  table.getEntry("targetPixelsY").getDouble(0);

   
    SmartDashboard.putNumber("Reef Distance", reefd);
   
    SmartDashboard.putNumber("Tag ID 1", tid1);
    
    SmartDashboard.putNumber("right Reef X", x1);
    
    SmartDashboard.putNumber("right Reef Y", y1);
    
    
      
      boolean isConnected = NetworkTableInstance.getDefault().isConnected();
SmartDashboard.putBoolean("NetworkTables Connected", isConnected);
  }
 /** Gets the detected AprilTag ID from the forward Limelight */
 public double getTagID() {
    
  return tid1;
}

public int getTargetYaw(){
  if(getTagID()==7.0||  getTagID() == 18.0 ){
    return 0;
  }
  else if(getTagID() == 17.0  ||  getTagID() == 8.0 ){
    return 60;
  }
  else if(getTagID() == 11.0  ||  getTagID() == 20.0){
    return -120;
  }
  else if(getTagID() == 6.0 ||  getTagID() == 19.0){
    return -60;
  }
  else if(getTagID() == 9.0  ||  getTagID() == 22.0 ){
    return 120;
  }
  else if(getTagID() == 10.0 ||  getTagID() == 21.0){
    if (s_Swerve.getPose().getRotation().getDegrees() >=0){
      return 180;}
      else {return -180;}
  }
  
  else if (getTagID() == 2.0  ||  getTagID() == 12.0 ){
    return 55;
  }
  else if (getTagID() == 1.0  ||  getTagID() == 13.0  ){
   return  -55;
  }
  else{
    return 0;
  }
}

}
