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
        public static double kPTurning = 0.5;

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

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kBackRightDriveMotorPort = 4; 

        public static final int kFrontLeftTurnMotorPort = 6;
        public static final int kFrontRightTurnMotorPort = 5;
        public static final int kBackLeftTurnMotorPort = 7;
        public static final int kBackRightTurnMotorPort = 8;

        public static final boolean kFrontLeftAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftAbsoluteEncoderReversed = true;
        public static final boolean kBackRightAbsoluteEncoderReversed = true;

        public static final double kFrontLeftAbsoluteEncoderOffset = Math.toRadians(60.732);
        public static final double kFrontRightAbsoluteEncoderOffset = Math.toRadians(40.166);
        public static final double kBackLeftAbsoluteEncoderOffset = Math.toRadians(-29.971);
        public static final double kBackRightAbsoluteEncoderOffset = Math.toRadians(-117.861);

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        public static final boolean kFrontLeftTurnMotorReversed = false;
        public static final boolean kFrontRightTurnMotorReversed = false;
        public static final boolean kBackLeftTurnMotorReversed = false;
        public static final boolean kBackRightTurnMotorReversed = false;

        public static boolean kFieldCentric = true;
        
        public static final int kFrontLeftAbsoluteEncoderPort = 14;
        public static final int kFrontRightAbsoluteEncoderPort = 13;
        public static final int kBackLeftAbsoluteEncoderPort = 15;
        public static final int kBackRightAbsoluteEncoderPort = 16;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = Math.toRadians(360);
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.toRadians(720);

        public static final double kV = 2.5253;
        public static final double kA = 1.4111;
        public static final double kS = 0.18974;
        public static final double kPVel = 4;
        public static final double kIVel = 0;
        public static final double kDVel = 0;

        public static final double kPTheta = 2.5;
        public static final double kPX = 3.5;
        public static final double kPY = 3.5;

        public static boolean slowMode = false;
    }

    public static final class ElevatorConstants {
        public static final double kS = 0.18789;
        public static final double kV = 10.349;
        public static final double kA = 0.50518;
        public static final double kG = 0.14103;

        public static final double kP = 1; 
        public static final double kI = 0.001;
        public static final double kD = 0.048;
        public static final double kRetractP = 0.374;
        public static final double kRetractI = 0.001;
        public static final double kRetractD = 0.05;

        public static final double kElevatorPulleyDiameter = 0.044; // in meters
        public static final double kElevatorMotorGearRatio = 1.0 / 12.0;
        public static final double kElevatorPulleyCircumference = Math.PI * kElevatorPulleyDiameter;
        public static final double kElevatorMotorTicksToM = kElevatorPulleyCircumference * kElevatorMotorGearRatio;
        public static final double kElevatorMotorRPMToMPerSec = kElevatorMotorTicksToM / 60;

        public static final double kElevatorMaxVel = 1;
        public static final double kElevatorMaxAccel = 1;

        public static final double kElevatorFeedBackPositionTolerance = 1;
        public static final double kElevatorFeedBackVelocityTolerance = 1;

        public static final double kElevatorPosCubeLow = 0;
        public static final double kElevatorPosCubeMid = 0.28;
        public static final double kElevatorPosCubeHigh = 0.54;
        
        public static final double kElevatorPosConeLow = 0.26;
        public static final double kElevatorPosConeMid = 0.55;
        public static final double kElevatorPosConeHigh = 0.75;

        public static final double kElevatorPosIntakeConeUp = 0.195;
        public static final double kElevatorPosIntakeConeTipped = 0.062;
        public static final double kElevatorPosIntakeConeSubstation = 0.097;
        public static final double kElevatorPosIntakeCubeGround = 0;
        public static final double kElevatorPosIntakeConeDoubleSubstation = 0.5;

        public static final double kElevatorPosHome = 0;

        public static boolean isHome = true;
        public static boolean isScore = false;
    }

public static final class IntakeConstants {
        public static final double kP = 2;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kVeloP = 0;
        public static final double kVeloI = 0;
        public static final double kVeloD = 0;

        public static final double kS = 0.28801;
        public static final double kV = 0.46192;
        public static final double kA = 0.038527;
        public static final double kCos = 0.95;

        public static final double kPivotMotorGearRatio = 1 / 22.5;
        public static final double kPivotMotorTicksToRadians = kPivotMotorGearRatio * Math.PI * 2;
        public static final double kPivotMotorRPMToRadiansPerSecond = kPivotMotorTicksToRadians / 60;
        public static final double kPivotEncoderOffset = Math.toRadians(0);

        public static final double kPivotFeedbackPositionTolerance = Math.toRadians(0.5);
        public static final double kPivotFeedbackVelocityTolerance = Math.toRadians(0.5);

        public static final double kPivotAngleRadScoreCube = Math.toRadians(60);
        public static final double kPivotAngleRadScoreConeMid = Math.toRadians(-40);
        public static final double kPivotAngleRadScoreConeHigh = Math.toRadians(-45);

        public static final double kPivotAngleRadIntakeConeUp = Math.toRadians(-45);
        public static final double kPivotAngleRadIntakeConeTipped = Math.toRadians(-57);
        public static final double kPivotAngleRadIntakeConeSubstation = Math.toRadians(60);
        public static final double kPivotAngleRadIntakeCubeGround = Math.toRadians(1);
        public static final double kPivotAngleRadIntakeConeDoubleSubstaion = Math.toRadians(45);

        public static final double kPivotAngleRadHome = Math.toRadians(90);

        public static final double kIntakeMaxVel = Math.toRadians(360);
        public static final double kIntakeMaxAccel = Math.toRadians(360);
    }


    public static final class CANConstants {                                                                                                                                                                              
        public static int kLeftElevatorMotor = 9;
        public static int kRightElevatorMotor = 10;
        public static int kIntakePivotMotor = 12;
        public static int kIntakeWheelMotor = 11;
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
