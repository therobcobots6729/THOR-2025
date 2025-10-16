package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
 
public final class Constants {
  public static final double stickDeadband = 0.12;
       public static final int warningTime = 4000;
       public static final int errorTime = 7000;
         public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
  public static final class Swerve {
    
    // public static final boolean invertGyro = true; // Always ensure Gyro is CCW+
    // CW-
    
    public static final int pigeonID = 23;
    public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(
            COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);
    
    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(26.5);
    public static final double wheelBase = Units.inchesToMeters(20);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
       // Elastic Notifications

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32;
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    
    /* Custom Multipliers */
    public static final double crabTurn = 0.8;
    public static final double driveLineup = 0.5;
    public static final double turnLineup = 0.25;
    
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // Originally 4.5, change if needed... Nah, FULL POWER

    /** Radians per Second */
    public static final double maxAngularVelocity = 8.0; // Originally 10.0

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 9;
      public static final int canCoderID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(107.66);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 13;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-137.46);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(175.65);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-105.38);////
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }



  

  /* Intake/Scoring Mechanism */
  public static final class sucky {

  }

  /* Wrist Mechanism */
  public static final class flippy {

  }

  /* Elevator Mechanism */
  public static final class extendy {
    public static final int spoolMotor = 22;
  }
  public static final class Vision {
    public static final double fieldBorderMargin = 0.25; // Reject poses this far outside the field
    public static final double maxZError = 0.5; // Reject poses this far above or below the floor
    public static final double autoAcceptAmbiguity = 0.1; // Automatically accept results with ambiguity less than this
    public static final double maxAmbiguity = 0.35; // Reject results with ambiguity greater than this

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.75; // Meters
    public static double angularStdDevBaseline = 2.0; // Radians

    public enum Camera {
        LEFT  ("AprilTag Left", new Transform3d(
            new Translation3d(Pose.robotFrameLength / 2.0 - Units.inchesToMeters(2.772),
                Pose.robotFrameWidth / 2.0 - Units.inchesToMeters(4.843),
                Units.inchesToMeters(8.46)),
            new Rotation3d(Units.degreesToRadians(1.3), Units.degreesToRadians(-15.5), Units.degreesToRadians(-30)))),
        CENTER("AprilTag Center", new Transform3d(
            new Translation3d(Pose.robotFrameLength / 2.0 - Units.inchesToMeters(6.958),
                0.0,
                Units.inchesToMeters(6.55)),
            new Rotation3d(Units.degreesToRadians(-0.6), Units.degreesToRadians(-19.2), Units.degreesToRadians(0)))),
        RIGHT ("AprilTag Right", new Transform3d(
            new Translation3d(Pose.robotFrameLength / 2.0 - Units.inchesToMeters(2.772),
                -Pose.robotFrameWidth / 2.0 + Units.inchesToMeters(4.843),
                Units.inchesToMeters(8.46)),
            new Rotation3d(Units.degreesToRadians(0.8), Units.degreesToRadians(-14.0), Units.degreesToRadians(30)))),
        REAR ("AprilTag Rear", new Transform3d(
            new Translation3d(Pose.robotFrameLength / 2.0 - Units.inchesToMeters(12.94),
                -Pose.robotFrameWidth / 2.0 + Units.inchesToMeters(2.75),
                Units.inchesToMeters(39.6)),
            new Rotation3d(0.0, Units.degreesToRadians(-8.5), Units.degreesToRadians(180.0))));
    
        Camera(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }

        public final String name;
        public final Transform3d robotToCamera;
    }

    public static final Camera[] camerasAvailable = Camera.values();
    // public static final Camera[] camerasAvailable = { Camera.CENTER };
}
public static final class Pose {
  

  public static final PIDController rotationPID = new PIDController(0.012, 0.0, 0.0); // kI was 0.050 for NCCMP 2024
  public static final double rotationKS = 0.02;
  public static final double rotationIZone = 2.0; // degrees

  public static final double tiltWarning = 10.0;
  public static final double tiltError = 30.0;

  // TODO What about AndyMark field?
  // NOTE That FlippingUtil might need to be impacted
  public static final double fieldWidth = FlippingUtil.fieldSizeY; // Units.inchesToMeters(26*12 + 5);
  public static final double fieldLength = FlippingUtil.fieldSizeX; // Units.inchesToMeters(57*12 + 6.875);

  public static final double reefElevatorZoneRadius = Units.inchesToMeters(80.0); // TODO Revisit
  public static final double autoUpDistance = Units.inchesToMeters(44.0); // Increase for quicker auto scoring, but risky
  public static final double wingLength = Units.inchesToMeters(280);

  public static final double robotFrameLength = Units.inchesToMeters(30);
  public static final double robotFrameWidth = Units.inchesToMeters(27);
  public static final double bumperWidth = Units.inchesToMeters(3.2);
  public static final double reefStandoff = Units.inchesToMeters(1.5);
  public static final double centerToFrontBumper = robotFrameLength / 2.0 + bumperWidth;
  public static final double reefOffset = centerToFrontBumper + reefStandoff;
  public static final double reefExtraOffset = Units.inchesToMeters(9.0);
  public static final double bonusStandoff = Units.inchesToMeters(4.0);

  
  // Locations from the Blue Alliance perspective
  public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.75), fieldWidth / 2.0);
  public static final double reefToFaceDistance = reefCenter.getX() - Units.inchesToMeters(144.0);
  public static final double branchSeparation = Units.inchesToMeters(12.0 + 15.0 / 16.0);
  public static final double bargeShotDistanceFromCenter = Units.inchesToMeters(52.0);
  public static final double bargeShotX = fieldLength / 2.0 - bargeShotDistanceFromCenter - centerToFrontBumper;

  // Offset to the reef face, not at the branches, but on the faces directly in front
  public static final Translation2d centerOffset = new Translation2d(reefToFaceDistance + reefOffset - reefStandoff, 0.0); // NOTE: Undo reef standoff for algae
  private static final Translation2d leftOffset = new Translation2d(reefToFaceDistance + reefOffset, -branchSeparation / 2.0);
  private static final Translation2d rightOffset = new Translation2d(reefToFaceDistance + reefOffset, branchSeparation / 2.0);
  private static final Translation2d extraOffset = new Translation2d(reefExtraOffset, 0.0);
  private static final Translation2d centerApproachOffset = centerOffset.plus(extraOffset);
  private static final Translation2d leftApproachOffset = leftOffset.plus(extraOffset);
  private static final Translation2d rightApproachOffset = rightOffset.plus(extraOffset);
  private static final Translation2d bonusOffset = new Translation2d(bonusStandoff, 0.0);
  private static final Translation2d leftBonusOffset = leftOffset.plus(bonusOffset);
  private static final Translation2d rightBonusOffset = rightOffset.plus(bonusOffset);
  private static final Transform2d leftL1Transform = new Transform2d(-Units.inchesToMeters(2.5), -branchSeparation / 2.0 + Units.inchesToMeters(2.0), Rotation2d.kZero);
  private static final Transform2d rightL1Transform = new Transform2d(-Units.inchesToMeters(2.5), branchSeparation / 2.0 - Units.inchesToMeters(2.0), Rotation2d.kZero);
  private static final Transform2d extraAlgaeBackup = new Transform2d(Units.inchesToMeters(-18.0), 0.0, Rotation2d.kZero);

  public static final double elevatorNoDownDistance = reefToFaceDistance + reefOffset + Units.inchesToMeters(12.0);

  public static enum ReefFace {
      AB(-180, true),
      CD(-120, false),
      EF(-60, true),
      GH(0, false),
      IJ(60, true),
      KL(120, false);

      ReefFace(double directionDegrees, boolean algaeHigh) {
          directionFromCenter = Rotation2d.fromDegrees(directionDegrees);
          alignMiddle = new Pose2d(reefCenter.plus(centerOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          alignLeft = new Pose2d(reefCenter.plus(leftOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          leftL1 = alignLeft.transformBy(leftL1Transform);
          alignRight = new Pose2d(reefCenter.plus(rightOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          rightL1 = alignRight.transformBy(rightL1Transform);
          approachMiddle = new Pose2d(reefCenter.plus(centerApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          algaeBackup = approachMiddle.plus(extraAlgaeBackup);
          approachLeft = new Pose2d(reefCenter.plus(leftApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          approachRight = new Pose2d(reefCenter.plus(rightApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          alignBonusLeft = new Pose2d(reefCenter.plus(leftBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          alignBonusRight = new Pose2d(reefCenter.plus(rightBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
          this.algaeHigh = algaeHigh;
      }

      public final Rotation2d directionFromCenter;
      public final Pose2d alignLeft, alignMiddle, alignRight;
      public final Pose2d leftL1, rightL1;
      public final Pose2d approachLeft, approachMiddle, approachRight;
      public final Pose2d algaeBackup;
      public final Pose2d alignBonusLeft, alignBonusRight;
      public final boolean algaeHigh;
  }

  // Cage locations from 6328
  public static enum Cage {
      CLOSE(199.947), // 5.079 m
      MIDDLE(242.855), // 6.169 m
      FAR(286.779), // 7.284 m
      HQ(8.16, 2.37);

      private final Translation2d location;
      private static final Translation2d redFudge = new Translation2d(0.08, 0.02);
      private static final Translation2d blueFudge = new Translation2d(0.0, 0.02);

      Cage(double yInches) {
          // 8.774 m
          location = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(yInches));
      }

      Cage(double xMeters, double yMeters) {
          location = new Translation2d(xMeters, yMeters);
      }

      public Translation2d location() {
          // NOTE: This is prior to flipping to red
          if (Robot.isRed()) {
              return location.plus(redFudge);
          } else {
              return location.plus(blueFudge);
          }        
      }
  }

  // 30 in / 2 + 3.2 in - 2.0 in = 16.2 in = 0.4115 m

  // fieldSizeX = Units.feetToMeters(57.573); 17.548 m
  // fieldSizeY = Units.feetToMeters(26.417); 8.052 m

  // Hypothetical
  // Blue CLOSE = 8.363, 5.08
  // Blue MIDDLE = 8.363, 6.17
  // Blue FAR = 8.363, 7.28
  // Red CLOSE = 9.19, 2.97
  // Red MIDDLE = 9.19, 1.883
  // Red FAR = 9.19, 0.768

  // Ashville
  // Want X diff of 0.30 m?
  // Blue CLOSE = 8.36 X 8.49, 5.23 X 5.09 
  // Blue MIDDLE = 8.47, 6.16
  // Blue FAR = 
  // Red CLOSE = 9.11, 2.90
  // Red MIDDLE = 9.07, 1.84
  // Red FAR = X

  // Mecklenburg
  // Blue CLOSE = 8.35, 5.10   (8.774 - 0.42)
  // Red CLOSE = 9.11, 2.95    (8.774 + 0.34)

  // public static final Transform2d cageOffset = new Transform2d(centerToFrontBumper - Units.inchesToMeters(2.0), 0, Rotation2d.kZero);
  public static final Transform2d cageOffset = new Transform2d(0.42, 0, Rotation2d.kZero);
  public static final Transform2d cageApproachOffset = new Transform2d(Units.inchesToMeters(16.0), 0, Rotation2d.kZero);
}
}
