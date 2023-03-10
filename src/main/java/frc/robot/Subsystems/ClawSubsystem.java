package frc.robot.Subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CANConstants;
import frc.robot.Constants.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor = new CANSparkMax(CANConstants.kClawPivotMotor, MotorType.kBrushless);
    private CANSparkMax wheelMotor = new CANSparkMax(CANConstants.kClawWheelMotor, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private PIDController pivotFeedback = new PIDController(ClawConstants.kP, ClawConstants.kI, ClawConstants.kD);
    private ArmFeedforward pivotFeedforward = new ArmFeedforward(ClawConstants.kS, ClawConstants.kCos, ClawConstants.kV, ClawConstants.kA);
    private BooleanSupplier isInterrupted;

    public ClawSubsystem() {
        pivotEncoder.setPositionConversionFactor(ClawConstants.kPivotMotorGearRatio);        
        pivotEncoder.setVelocityConversionFactor(ClawConstants.kPivotMotorTicksToRadians);

        pivotEncoder.setPosition(ClawConstants.kPivotEncoderOffset);

        pivotFeedback.setIntegratorRange(-1, 1);
        pivotFeedback.setTolerance(ClawConstants.kPivotFeedbackPositionTolerance, ClawConstants.kPivotFeedbackVelocityTolerance);
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Pivot Motor Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Claw Pivot Target Position", pivotEncoder.getPosition());
    }

    public void setPosition(double clawPosition) {
        pivotFeedback.setSetpoint(clawPosition);
    }

    public void setMotorVoltageByPosition(double pivotAngleRad) {
        pivotMotor.setVoltage(pivotFeedforward.calculate(pivotAngleRad, 0) + pivotFeedback.calculate(pivotEncoder.getPosition()));
    }

    public void setPivotPower(double pivotPower) {
        pivotMotor.set(pivotPower);
    }

    public void setWheelPower(double wheelPower) {
        wheelMotor.set(wheelPower);
    }

    public void goToPosition(double pivotAngleRad) {
        setPosition(pivotAngleRad);

        while(!pivotFeedback.atSetpoint() && !isInterrupted.getAsBoolean()) {
            setMotorVoltageByPosition(pivotAngleRad);

            //to not hog CPU cycles
            try {
                Thread.sleep(25);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
