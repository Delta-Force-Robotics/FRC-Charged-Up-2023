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
    private ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kV, ArmConstants.kCos, ArmConstants.kA);
    private PIDController leftArmFeedback = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private PIDController rightArmFeedback = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    private BooleanSupplier isInterrupted;

    public ArmSubsystem() {
        rightMotor.setInverted(true);

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

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Position", Math.toDegrees(leftEncoder.getPosition()));
        SmartDashboard.putNumber("Right Arm Position", Math.toDegrees(rightEncoder.getPosition()));
        SmartDashboard.putNumber("Arm Target Position", leftArmFeedback.getSetpoint());
    }

    public double[] getPosition() {
        return new double[]{ leftEncoder.getPosition(), rightEncoder.getPosition() };
    }

    public void setPosition(double armAngleRad) {
        leftArmFeedback.setSetpoint(armAngleRad);
        rightArmFeedback.setSetpoint(armAngleRad);
    }

    public void setMotorVoltageByPosition(double armAngleRad) {
        leftMotor.setVoltage(armFeedforward.calculate(armAngleRad, 0) + leftArmFeedback.calculate(leftEncoder.getPosition()));
        rightMotor.setVoltage(armFeedforward.calculate(armAngleRad, 0) + rightArmFeedback.calculate(rightEncoder.getPosition()));
    }

    public void setMotorPower(double motorPower) {
        leftMotor.set(motorPower);
        rightMotor.set(motorPower);
    }

    public void goToPosition(double armAngleRad) {
        setPosition(armAngleRad);

        while(!leftArmFeedback.atSetpoint() && !rightArmFeedback.atSetpoint() && !isInterrupted.getAsBoolean()) {
            setMotorVoltageByPosition(armAngleRad);

            //to not hog CPU cycles
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
