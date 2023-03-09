package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(Constants.CANConstants.kLeftArmMotor, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CANConstants.kRightArmMotor, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kV, ArmConstants.kCos, ArmConstants.kA);
    private PIDController leftArmFeedback = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private PIDController rightArmFeedback = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);

    public ArmSubsystem() {
        leftEncoder.setPositionConversionFactor(ArmConstants.kArmMotorTicksToRadians);
        rightEncoder.setPositionConversionFactor(ArmConstants.kArmMotorTicksToRadians);
        
        leftEncoder.setVelocityConversionFactor(ArmConstants.kArmMotorRPMToRadiansPerSec);
        rightEncoder.setVelocityConversionFactor(ArmConstants.kArmMotorRPMToRadiansPerSec);

        leftEncoder.setPosition(ArmConstants.kArmEncoderOffset);

        rightArmFeedback.setIntegratorRange(-1, 1);
        leftArmFeedback.setIntegratorRange(-1, 1);

        rightArmFeedback.setTolerance(ArmConstants.kArmFeedBackPositionTolerance, ArmConstants.kArmFeedBackVelocityTolerance);
        leftArmFeedback.setTolerance(ArmConstants.kArmFeedBackPositionTolerance, ArmConstants.kArmFeedBackVelocityTolerance);
    }

    public void setPosition(double armAngleRad) {
        leftArmFeedback.setSetpoint(armAngleRad);
        rightArmFeedback.setSetpoint(armAngleRad);
    }

    public void calculate(double armAngleRad) {
        leftMotor.setVoltage(armFeedforward.calculate(armAngleRad, 0) + leftArmFeedback.calculate(leftEncoder.getPosition()));
        rightMotor.setVoltage(armFeedforward.calculate(armAngleRad, 0) + leftArmFeedback.calculate(rightEncoder.getPosition()));
    }
}
