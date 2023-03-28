package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax leftMotor = new CANSparkMax(Constants.CANConstants.kLeftArmMotor, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CANConstants.kRightArmMotor, MotorType.kBrushless);
    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kCos, ArmConstants.kV, ArmConstants.kA);
    private PIDController leftArmFeedback = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private PIDController rightArmFeedback = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private BooleanSupplier isInterrupted;
    private ArmPosition desiredArmPosition = ArmPosition.HOME;
    private ArmPosition currArmPosition = ArmPosition.HOME;
    private double setPoint = ArmConstants.kArmEncoderOffset;

    public ArmSubsystem() {
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        
        leftEncoder.setPositionConversionFactor(ArmConstants.kArmMotorTicksToRadians);
        rightEncoder.setPositionConversionFactor(ArmConstants.kArmMotorTicksToRadians);

        leftEncoder.setVelocityConversionFactor(ArmConstants.kArmMotorRPMToRadiansPerSec);
        rightEncoder.setVelocityConversionFactor(ArmConstants.kArmMotorRPMToRadiansPerSec);

        leftEncoder.setPosition(ArmConstants.kArmEncoderOffset);
        rightEncoder.setPosition(ArmConstants.kArmEncoderOffset);

        rightArmFeedback.setIntegratorRange(-1, 1);
        leftArmFeedback.setIntegratorRange(-1, 1);

        rightArmFeedback.setTolerance(ArmConstants.kArmFeedBackPositionTolerance,
                ArmConstants.kArmFeedBackVelocityTolerance);
        leftArmFeedback.setTolerance(ArmConstants.kArmFeedBackPositionTolerance,
                ArmConstants.kArmFeedBackVelocityTolerance);
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Position", Math.toDegrees(leftEncoder.getPosition()));
        SmartDashboard.putNumber("Right Arm Position", Math.toDegrees(rightEncoder.getPosition()));
        SmartDashboard.putNumber("Arm Target Position", Math.toDegrees(setPoint));
        SmartDashboard.putNumber("Motor Power", leftMotor.getAppliedOutput());
        
        ArmConstants.armAngle = leftEncoder.getPosition();
        setMotorVoltageByPosition(setPoint);
    }

    public double[] getPosition() {
        return new double[] { leftEncoder.getPosition(), rightEncoder.getPosition() };
    }

    public double getSetPoint() {
        return setPoint;
    }

    public void setSetpoint(double armAngleRad) {
        if(setPoint - armAngleRad >= 0) {
            leftArmFeedback.setP(ArmConstants.kRetractP);
            rightArmFeedback.setP(ArmConstants.kRetractP);

            leftArmFeedback.setD(ArmConstants.kRetractD);
            rightArmFeedback.setD(ArmConstants.kRetractD);
        }
        else {
            leftArmFeedback.setP(ArmConstants.kP);
            rightArmFeedback.setP(ArmConstants.kP);

            leftArmFeedback.setD(ArmConstants.kD);
            rightArmFeedback.setD(ArmConstants.kD);
        }

        setPoint = armAngleRad;
    }

    public void setMotorVoltageByPosition(double armAngleRad) {
        leftMotor.setVoltage(
                armFeedforward.calculate(armAngleRad, 0) + leftArmFeedback.calculate(leftEncoder.getPosition(), armAngleRad));
        rightMotor.setVoltage(
                armFeedforward.calculate(armAngleRad, 0) + rightArmFeedback.calculate(rightEncoder.getPosition(), armAngleRad));
    }

    public double getArmPosition() {
        return leftEncoder.getPosition();
    }

    public double getFeedForwardPower() {
        return armFeedforward.calculate(getArmPosition(), 0);
    }

    public void setMotorPower(double motorPower) {
        leftMotor.set(motorPower);
        rightMotor.set(motorPower);
    }

    public void setMotorVoltage(double motorVoltage) {
        leftMotor.setVoltage(motorVoltage);
        rightMotor.setVoltage(motorVoltage);
    }

    public ArmPosition getDesiredArmPosition() {
        return desiredArmPosition;
    }

    public void setCurrArmState(ArmPosition armPosition) {
        currArmPosition = armPosition;
    }

    public void setDesiredArmPosition(ArmPosition currArmPosition) {
        this.desiredArmPosition = currArmPosition;
    }

    public ArmPosition getCurrArmState() {
        return currArmPosition;
    }

    public static enum ArmPosition {
        HOME,
        INTAKE,
        LOW,
        MID,
        HIGH;
    }
}
