package frc.robot.Subsystems;

import java.io.Console;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Constants.CANConstants;
import frc.robot.Constants.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private CANSparkMax pivotMotor = new CANSparkMax(CANConstants.kIntakePivotMotor, MotorType.kBrushless);
    private CANSparkMax wheelMotor = new CANSparkMax(CANConstants.kIntakeWheelMotor, MotorType.kBrushless);
    private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
    private PIDController pivotFeedBack = new PIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD);
    private ArmFeedforward pivotFeedForward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kCos, IntakeConstants.kV, IntakeConstants.kA);
    //private DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(0);
    private AbsoluteEncoder absoluteEncoderPivot = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle); 

    private BooleanSupplier isInterrupted;

    private double setPoint = Math.toRadians(0); 

    public WheelDirection currWheelDirection = WheelDirection.OFF;
    private GameElement currGameElement = GameElement.CONE;

    public IntakeSubsystem() {
        pivotMotor.setInverted(true);    

        pivotEncoder.setPositionConversionFactor(IntakeConstants.kPivotMotorTicksToRadians);        
        pivotEncoder.setVelocityConversionFactor(IntakeConstants.kPivotMotorRPMToRadiansPerSecond);

        //pivotEncoder.setPosition(Math.toRadians(100));
        absoluteEncoderPivot.setInverted(true);

        absoluteEncoderPivot.setPositionConversionFactor(IntakeConstants.kPivotMotorTicksToRadians);
        absoluteEncoderPivot.setVelocityConversionFactor(IntakeConstants.kPivotMotorRPMToRadiansPerSecond);

        pivotEncoder.setPosition(absoluteEncoderPivot.getPosition());

        pivotFeedBack.setIntegratorRange(-1, 1);

        /*throughBoreEncoder.setPositionOffset(0.32);        
        throughBoreEncoder.setDistancePerRotation(2 * Math.PI);*/
    }

    public void setInterruptedCondition(BooleanSupplier isInterrupted) {
        this.isInterrupted = isInterrupted;
    }

    /*public double getPosition() {
        return absoluteEncoderPivot.getPosition();
    } */

    /*public double getPosition() {
        return wrapAroundEncoder((-throughBoreEncoder.getAbsolutePosition() + throughBoreEncoder.getPositionOffset()) * throughBoreEncoder.getDistancePerRotation());
    }*/

    /*public double wrapAroundEncoder(double encoderPosition) {
        if(encoderPosition > 0) {
            return encoderPosition;
        }

        return throughBoreEncoder.getDistancePerRotation() + encoderPosition;
    }*/

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivot", pivotEncoder.getPosition());        //SmartDashboard.putNumber("pivot motor", pivotEncoder.getPosition());
        SmartDashboard.putNumber("absolut", absoluteEncoderPivot.getPosition());
        SmartDashboard.putNumber("feedforward", pivotFeedForward.calculate(IntakeConstants.kPivotAngleRadIntakeConeSubstation, 0));
        SmartDashboard.putNumber("feedback", pivotFeedBack.calculate(pivotEncoder.getPosition(), IntakeConstants.kPivotAngleRadIntakeConeSubstation));
        SmartDashboard.putNumber("feedforwardback", pivotFeedForward.calculate(IntakeConstants.kPivotAngleRadIntakeConeSubstation, 0) + pivotFeedBack.calculate(pivotEncoder.getPosition(), IntakeConstants.kPivotAngleRadIntakeConeSubstation));
        SmartDashboard.putNumber("offset", absoluteEncoderPivot.getZeroOffset());

        updateWheelPower();

        setMotorVoltageByPosition(setPoint);
    }

    /*public void scheduleGains(double pivotPosition) {
        pivotFeedBack.setP(Math.abs(Math.cos(pivotPosition)) * IntakeConstants.kP);
    }*/

    public void setMotorVoltageByPosition(double pivotAngleRad) {
        pivotMotor.setVoltage(pivotFeedBack.calculate(pivotFeedForward.calculate(pivotAngleRad, 0) + pivotEncoder.getPosition(), pivotAngleRad));
    }
    
    public void setPivotPower(double pivotPower) {
        pivotMotor.set(pivotPower);
    }

    public void updateWheelPower() {
        double wheelPower = 0;

        if(currWheelDirection == WheelDirection.INTAKE) {
            wheelPower = 0.5;
        }
        else if(currWheelDirection == WheelDirection.OUTTAKE) {
            wheelPower = -0.2;
        }

        wheelMotor.set(wheelPower);
    }

    public void setWheelDirection(WheelDirection wDirection) {
        currWheelDirection = wDirection;
    }

    public WheelDirection getWheelDirection() {
        return currWheelDirection;
    }

    public void setCurrGameElement(GameElement currGameElement) {
        this.currGameElement = currGameElement;
    }

    public GameElement getCurrGameElement() {
        return currGameElement;
    }

    public void setSetpoint(double intakeAngleRad) {
        setPoint = intakeAngleRad;
    }

    public static enum GameElement {
        CUBE,
        CONE
    };

    public static enum WheelDirection {
        INTAKE,
        OUTTAKE,
        OFF
    }
}
