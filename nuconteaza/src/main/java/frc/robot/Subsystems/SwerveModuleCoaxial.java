
package frc.robot.Subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.SwerveModuleConstants;

public class SwerveModuleCoaxial {

    public final CANSparkMax driveMotor;
    public final CANSparkMax turnMotor;

    public final RelativeEncoder driveEncoder;
    public final RelativeEncoder turnEncoder;

    private final PIDController turningPidController;

    public final CANCoder absoluteEncoder;
    public final boolean turnEncoderReversed;
    public final double turnEncoderOffset;

    public SwerveModuleCoaxial (int driveMotorID, int turnMotorID, int turnEncoderID, int driveEncoderID, boolean turnEncoderReversed, double turnEncoderOffset, 
                boolean driveMotorReversed, boolean turnMotorReversed, int absoluteEncoderID) {
        this.turnEncoderReversed = turnEncoderReversed;
        this.turnEncoderOffset = turnEncoderOffset;
        absoluteEncoder = new CANCoder(absoluteEncoderID);
        
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turnMotor.setInverted(turnMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kdriveEncoderRPM2MeterPerSec);

        turnEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(SwerveModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
    }

    public double getDrivePosition () {
        return driveEncoder.getPosition();
    }

    public double getTurnPosition () {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity () {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity () {
        return turnEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        return 0;

    }
}