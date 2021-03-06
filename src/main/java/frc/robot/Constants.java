package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class Constants {
    public static final class DriveConstants {
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kRearLeftDriveMotorPort = 3;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kRearRightDriveMotorPort = 4;
    
        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kRearLeftTurningMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 12;
        public static final int kRearRightTurningMotorPort = 14;
    
         // public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    // public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    // public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    // public static final int[] kRearRightTurningEncoderPorts = new int[] {5, 6};

    // public static final boolean kFrontLeftTurningEncoderReversed = false;
    // public static final boolean kRearLeftTurningEncoderReversed = true;
    // public static final boolean kFrontRightTurningEncoderReversed = false;
    // public static final boolean kRearRightTurningEncoderReversed = true;

    // public static final int[] kFrontLeftDriveEncoderPorts = new int[] {7, 8};
    // public static final int[] kRearLeftDriveEncoderPorts = new int[] {9, 10};
    // public static final int[] kFrontRightDriveEncoderPorts = new int[] {11, 12};
    // public static final int[] kRearRightDriveEncoderPorts = new int[] {13, 14};

    // public static final boolean kFrontLeftDriveEncoderReversed = false;
    // public static final boolean kRearLeftDriveEncoderReversed = true;
    // public static final boolean kFrontRightDriveEncoderReversed = false;
    // public static final boolean kRearRightDriveEncoderReversed = true;

    public static final double kTrackWidth = 0.53975; //0.5;
    // Distance between centers of right and left wheels on robot in meters
    public static final double kWheelBase = 0.5969; //0.7;
    // Distance between front and back wheels on robot in meters
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

            public static final boolean kGyroReversed = false;

            // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
            // These characterization values MUST be determined either experimentally or theoretically
            // for *your* robot's drive.
            // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
            // values for your robot.
            public static final double ksVolts = 1;
            public static final double kvVoltSecondsPerMeter = 0.8;
            public static final double kaVoltSecondsSquaredPerMeter = 0.15;
        
            public static final double kMaxSpeedMetersPerSecond = 4.93776; // 3
          }
          

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.93776; // this is the hardware maximum, it might want to be tuned down
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.93776;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
    
}
