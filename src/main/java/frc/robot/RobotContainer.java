package frc.robot;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.ArmManualControl;
import frc.robot.Commands.SwerveJoystickCommand;
import frc.robot.Constants.Constants;
import frc.robot.Constants.Constants.ArmConstants;
import frc.robot.Constants.Constants.ClawConstants;
import frc.robot.Constants.Constants.DriveConstants;
import frc.robot.Constants.Constants.OIConstants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ClawSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.ArmSubsystem.ArmPosition;
import frc.robot.Subsystems.ClawSubsystem.GameElement;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.CamMode;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.LEDState;
import frc.robot.Threads.ArmGoToPosition;
import frc.robot.Threads.ClawGoToPosition;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limeLightSubsystem = new LimelightSubsystem("tx", "ty", "ta");
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick driver2Joystick = new Joystick(OIConstants.kDriver2ControllerPort);

    private final SendableChooser autoStartSide = new SendableChooser<String>();
    private final SendableChooser autoRoutineChooser = new SendableChooser<String>();

    private final ArmGoToPosition armThread = new ArmGoToPosition(armSubsystem);
    private final ClawGoToPosition clawThread = new ClawGoToPosition(clawSubsystem, Math.toRadians(0));

    private final Consumer<Double[]> scoreExecutor = (Double[] positions) -> {
        
    };

    public RobotContainer() {
        autoStartSide.addOption("Blue", "Blue");
        autoStartSide.addOption("Red", "Red");

        autoRoutineChooser.addOption("Preload Park", "AutoMid");
        autoRoutineChooser.addOption("Preload 1 Park", "AutoBottom");
        autoRoutineChooser.addOption("Preload 2 Park", "AutoTop");

        SmartDashboard.putData(autoStartSide);
        SmartDashboard.putData(autoRoutineChooser);
    }

    public void teleOpInit() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        armSubsystem.setDefaultCommand(new ArmManualControl(armSubsystem,
            () -> driverJoystick.getRawAxis(4),
            () -> driverJoystick.getRawAxis(3)
        ));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(driverJoystick, 2)
                .onTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));

        new POVButton(driverJoystick, 270).onTrue(Commands.runOnce(() -> {
            if (limeLightSubsystem.currCamMode == CamMode.VISION_PROCESSOR) {
                limeLightSubsystem.setCamMode(CamMode.DRIVER_CAMERA);
            } else {
                limeLightSubsystem.setCamMode(CamMode.VISION_PROCESSOR);
            }
        }, limeLightSubsystem));

        new POVButton(driverJoystick, 90).onTrue(Commands.runOnce(() -> {
            if (limeLightSubsystem.currLedState == LEDState.PIPELINE) {
                limeLightSubsystem.setLedMode(LEDState.BLINK);
            } else {
                limeLightSubsystem.setLedMode(LEDState.PIPELINE);
            }
        }, limeLightSubsystem));

        new JoystickButton(driver2Joystick, 1).onTrue(Commands.runOnce(() -> {
            armSubsystem.setDesiredArmPosition(ArmSubsystem.ArmPosition.LOW);
        }));

        new JoystickButton(driver2Joystick, 2).onTrue(Commands.runOnce(() -> {
            armSubsystem.setDesiredArmPosition(ArmSubsystem.ArmPosition.MID);
        }));

        new JoystickButton(driver2Joystick, 3).onTrue(Commands.runOnce(() -> {
            armSubsystem.setDesiredArmPosition(ArmSubsystem.ArmPosition.HIGH);
        }));

        new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            if(armSubsystem.getIsHome()) {

                new Thread(() -> {
                    clawSubsystem.goToPosition(ClawConstants.clawAngleRadIntake);
                    clawSubsystem.setWheelDirection(ClawSubsystem.WheelDirection.INTAKE);
                }).start();
    
                new Thread(() -> {
                    try {
                        armSubsystem.goToPosition(ArmConstants.armAngleRadIntake);
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }).start();
            }
            else {

                new Thread(() -> {
                    clawSubsystem.goToPosition(Constants.ClawConstants.kPivotEncoderOffset);
                    clawSubsystem.setWheelDirection(ClawSubsystem.WheelDirection.OFF);
                }).start();
    
                new Thread(() -> {
                    try {
                        armSubsystem.goToPosition(ArmConstants.armAngleRadHome);
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }).start();
            }
        }));

        new JoystickButton(driverJoystick, 5).onTrue(Commands.runOnce(() -> {
            if(!ArmConstants.isScore) {

                new Thread(() -> {
                    clawSubsystem.goToPosition(getPositionsToScore()[1]);
                }).start();
    
                new Thread(() -> {
                    try {
                        armSubsystem.goToPosition(getPositionsToScore()[0]);
                    } catch (InterruptedException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }).start();

                ArmConstants.isScore = true;
            }
            else {
                new Thread(() -> {
                    clawSubsystem.setWheelDirection(ClawSubsystem.WheelDirection.OUTTAKE);
                }).start();

                ArmConstants.isScore = false;
            }
        }));

        new JoystickButton(driverJoystick, 4).onTrue(Commands.runOnce(() -> {

            new Thread(() -> {
                clawSubsystem.goToPosition(Math.toRadians(45));
            }).start();

            new Thread(() -> {
                try {
                    armSubsystem.goToPosition(Math.toRadians(-45));
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }).start();
        }));
    }

    public double[] getPositionsToScore() {
        if (clawSubsystem.getCurrGameElement() == GameElement.CONE) {
            if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.LOW) {
                return new double[] { Constants.ArmConstants.armAngleRadConeLow,
                        Constants.ClawConstants.clawAngleRadScoreCone - Constants.ArmConstants.armAngleRadConeLow };
            } else if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.MID) {
                return new double[] { Constants.ArmConstants.armAngleRadConeMid,
                        Constants.ClawConstants.clawAngleRadScoreCone - Constants.ArmConstants.armAngleRadConeMid };
            } else {
                return new double[] { Constants.ArmConstants.armAngleRadConeHigh,
                        Constants.ClawConstants.clawAngleRadScoreCone - Constants.ArmConstants.armAngleRadConeHigh };
            }
        } else {
            if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.LOW) {
                return new double[] { Constants.ArmConstants.armAngleRadCubeLow,
                        Constants.ClawConstants.clawAngleRadScoreCube - Constants.ArmConstants.armAngleRadCubeLow };
            }
            if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.MID) {
                return new double[] { Constants.ArmConstants.armAngleRadCubeMid,
                        Constants.ClawConstants.clawAngleRadScoreCube - Constants.ArmConstants.armAngleRadCubeMid };
            } else {
                return new double[] { Constants.ArmConstants.armAngleRadCubeHigh,
                        Constants.ClawConstants.clawAngleRadScoreCube - Constants.ArmConstants.armAngleRadCubeHigh };
            }
        }
    }

    public Command getAutonomousCommand() {
        Pose2d startPose, outOfCommunityPose, endPose;
        if(autoStartSide.getSelected() == "Red") {
            startPose = new Pose2d(new Translation2d(14.837, 3.309), Rotation2d.fromDegrees(0));
        }
        else {
            startPose = new Pose2d(new Translation2d(1.587, 3.309), Rotation2d.fromDegrees(180));
        }

        outOfCommunityPose = new Pose2d(new Translation2d(startPose.getX() - Math.cos(startPose.getRotation().getRadians()) * 5, startPose.getY()), startPose.getRotation());
        endPose = new Pose2d(new Translation2d(12.2, 2), Rotation2d.fromDegrees(0));

        swerveSubsystem.setOdometryPosition(startPose);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveConstants.kPhysicalMaxSpeedMetersPerSecond, DriveConstants.kPhysicalMaxAccelMetersPerSecondSq)
                .setKinematics(DriveConstants.kDriveKinematics);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startPose, 
        List.of(), //List.of(outOfCommunityPose.getTranslation()),
        endPose, 
        trajectoryConfig);
        ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.kPTheta, 0, 0, new Constraints(DriveConstants.kPhysicalMaxAngSpeedRadPerSecond, DriveConstants.kPhysicalMaxAngAccelRadPerSecondSq));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PIDController xController = new PIDController(DriveConstants.kPX, 0, 0);
        PIDController yController = new PIDController(DriveConstants.kPY, 0, 0);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
            swerveSubsystem::getPose2d,
            DriveConstants.kDriveKinematics,
            new HolonomicDriveController(xController, yController, thetaController),
            swerveSubsystem::setModulesStates,
            swerveSubsystem
        );

        return swerveControllerCommand.andThen(() -> swerveSubsystem.stopModules());
    }
}