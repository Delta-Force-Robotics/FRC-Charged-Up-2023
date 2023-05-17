package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class AutoConstants {
        public static final Pose2d kLeftStartPose = new Pose2d(new Translation2d(-1, -1), new Rotation2d(Math.toRadians(-1)));
        public static final Pose2d kMidStartPose = new Pose2d(new Translation2d(-1, -1), new Rotation2d(Math.toRadians(-1)));
        public static final Pose2d kRightStartPose = new Pose2d(new Translation2d(-1, -1), new Rotation2d(Math.toRadians(-1)));
    }

    public static final class SwerveModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 21.4285;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static double kPTurning = 0.6;

        public static final double kCPRAbsoluteEncoder = 4096;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(21.805);
        public static final double kWheelBase = Units.inchesToMeters(22.909);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2)
        );

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAccelMetersPerSecondSq = 5;
        public static final double kPhysicalMaxAngSpeedRadPerSecond = Math.toRadians(180);
        public static final double kPhysicalMaxAngAccelRadPerSecondSq = Math.toRadians(180);

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 5; 

        public static final int kFrontLeftTurnMotorPort = 10;
        public static final int kFrontRightTurnMotorPort = 2;
        public static final int kBackLeftTurnMotorPort = 9;
        public static final int kBackRightTurnMotorPort = 4;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftAbsoluteEncoderReversed = true;
        public static final boolean kBackRightAbsoluteEncoderReversed = true;

        public static final double kFrontLeftAbsoluteEncoderOffset = Math.toRadians(58.975);
        public static final double kFrontRightAbsoluteEncoderOffset = Math.toRadians(147.041);
        public static final double kBackLeftAbsoluteEncoderOffset = Math.toRadians(-140.889);
        public static final double kBackRightAbsoluteEncoderOffset = Math.toRadians(-117.861);

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        public static final boolean kFrontLeftTurnMotorReversed = false;
        public static final boolean kFrontRightTurnMotorReversed = false;
        public static final boolean kBackLeftTurnMotorReversed = false;
        public static final boolean kBackRightTurnMotorReversed = false;

        public static boolean kFieldCentric = false;
        
        public static final int kFrontLeftAbsoluteEncoderPort = 15;
        public static final int kFrontRightAbsoluteEncoderPort = 14;
        public static final int kBackLeftAbsoluteEncoderPort = 16;
        public static final int kBackRightAbsoluteEncoderPort = 13;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = Math.toRadians(360);
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.toRadians(360);

        public static final double kV = 2.6284;
        public static final double kA = 0.1677;
        public static final double kS = 0.19357;
        public static final double kPVel = 2;
        public static final double kIVel = 0;
        public static final double kDVel = 0;

        public static final double kPTheta = 4;
        public static final double kPX = 4;
        public static final double kPY = 4;

        public static boolean slowMode = false;
    }

    public static final class ElevatorConstants {
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kCos = 0;

        public static final double kP = 0;
        public static final double kRetractP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kRetractD = 0;
        public static final double kRetractI = 0;

        public static final double kElevatorPulleyDiameter = 1; // in cm
        public static final double kElevatorMotorGearRatio = 1.0 / 80.0;
        public static final double kElevatorPulleyCircumference = Math.PI * kElevatorPulleyDiameter;
        public static final double kElevatorMotorTicksToCm = kElevatorMotorGearRatio * kElevatorPulleyCircumference;
        public static final double kElevatorMotorRPMToCmPerSec = kElevatorMotorTicksToCm / 60;

        public static final double kElevatorFeedBackPositionTolerance = 1;
        public static final double kElevatorFeedBackVelocityTolerance = 1;

        public static final double kElevatorPosCubeLow = 0;
        public static final double kElevatorPosCubeMid = 0;
        public static final double kElevatorPosCubeHigh = 0;
        
        public static final double kElevatorPosConeLow = 0;
        public static final double kElevatorPosConeMid = 0;
        public static final double kElevatorPosConeHigh = 0;

        public static final double kElevatorPosIntake = 0;
        public static final double kElevatorPosHome = 0;
    }

    public static final class IntakeConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;
        public static final double kCos = 0;

        public static final double kPivotMotorGearRatio = 0;
        public static final double kPivotMotorTicksToRadians = kPivotMotorGearRatio * Math.PI * 2;
        public static final double kPivotMotorRPMToRadiansPerSecond = kPivotMotorTicksToRadians / 60;
        public static final double kPivotEncoderOffset = Math.toRadians(0);

        public static final double kPivotFeedbackPositionTolerance = Math.toRadians(0.5);
        public static final double kPivotFeedbackVelocityTolerance = Math.toRadians(0.5);

        public static final double kPivotAngleRadScoreCube = Math.toRadians(0);
        public static final double kPivotAngleRadScoreCone = Math.toRadians(0);
        public static final double kPivotAngleRadIntake = Math.toRadians(0);
        public static final double kPivotAngleRadGroundIntake = Math.toRadians(0);

        public static final double kAngleRadHome = Math.toRadians(0);
        public static double kAngleFeedbackHome = Math.toRadians(0);
    }

    public static final class CANConstants {                                                                                                                                                                              
        public static int kLeftElevatorMotor = 0;
        public static int kRightElevatorMotor = 0;
        public static int kIntakePivotMotor = 0;
        public static int kIntakeWheelMotor = 0;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriver2ControllerPort = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 4;

        public static final double kDeadband = 0.1;
    }
}
