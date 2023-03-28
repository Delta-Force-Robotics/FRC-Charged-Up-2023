package frc.robot;

import java.util.List;
import java.util.function.Consumer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.Subsystems.ClawSubsystem.WheelDirection;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.CamMode;
import frc.robot.Subsystems.LimelightSubsystem.LimelightOptions.LEDState;

public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final LimelightSubsystem limeLightSubsystem = new LimelightSubsystem("tx", "ty", "ta");
    public final ClawSubsystem clawSubsystem = new ClawSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick driver2Joystick = new Joystick(OIConstants.kDriver2ControllerPort);
    private boolean configured = false;

    private final SendableChooser autoStartSideChooser = new SendableChooser<String>();
    private final SendableChooser autoRoutineChooser = new SendableChooser<String>();


    public RobotContainer() {
        autoStartSideChooser.addOption("Blue", "BlueSide");
        autoStartSideChooser.addOption("Red", "RedSide");

        autoRoutineChooser.addOption("Mid", "AutoMid");
        autoRoutineChooser.addOption("Cable Cover", "AutoCableCover");
        autoRoutineChooser.addOption("Open", "AutoOpen");

        SmartDashboard.putData(autoStartSideChooser);
        SmartDashboard.putData(autoRoutineChooser);
        clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
    }

    public void teleOpInit() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCommand(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> DriveConstants.kFieldCentric));


        configureButtonBindings();
    }

    private void configureButtonBindings() {
        configured = true;

        new JoystickButton(driverJoystick, 1)
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

        new JoystickButton(driver2Joystick, 5).onTrue(Commands.runOnce(() -> {
            clawSubsystem.setCurrGameElement(GameElement.CUBE);
        }, clawSubsystem));

        new JoystickButton(driver2Joystick, 6).onTrue(Commands.runOnce(() -> {
            clawSubsystem.setCurrGameElement(GameElement.CONE);;
        }, clawSubsystem));

        new JoystickButton(driver2Joystick, 2).onTrue(Commands.runOnce(() -> {
            armSubsystem.setDesiredArmPosition(ArmSubsystem.ArmPosition.LOW);
        }));

        new JoystickButton(driver2Joystick, 3).onTrue(Commands.runOnce(() -> {
            armSubsystem.setDesiredArmPosition(ArmSubsystem.ArmPosition.MID);
        }));

        new JoystickButton(driver2Joystick, 4).onTrue(Commands.runOnce(() -> {
            armSubsystem.setDesiredArmPosition(ArmSubsystem.ArmPosition.HIGH);
        }));

        new JoystickButton(driverJoystick, 6).onTrue(Commands.runOnce(() -> {
            if(ArmConstants.isHome) {
                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadIntake);
                clawSubsystem.setWheelDirection(ClawSubsystem.WheelDirection.INTAKE);
    

                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadIntake);

                ArmConstants.isScore = false;
                ArmConstants.isHome = false;
            }
            else {
                clawSubsystem.setSetPoint(Constants.ClawConstants.kPivotEncoderOffset);
                clawSubsystem.setWheelDirection(ClawSubsystem.WheelDirection.OFF);    

                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);

                ArmConstants.isScore = false;
                ArmConstants.isHome = true;
            }
        }, armSubsystem, clawSubsystem));

        new JoystickButton(driverJoystick, 5).onTrue(Commands.runOnce(() -> {
            if(!ArmConstants.isScore) {
                clawSubsystem.setSetPoint(getPositionsToScore()[1]);
                armSubsystem.setSetpoint(getPositionsToScore()[0]);

                ArmConstants.isScore = true;
                ArmConstants.isHome = false;
            }
            else {
                if(clawSubsystem.getCurrGameElement() == GameElement.CONE) {
                    clawSubsystem.setSetPoint(0);
                    armSubsystem.setSetpoint(armSubsystem.getSetPoint() - Math.toRadians(4));
                }
                
                clawSubsystem.setWheelDirection(ClawSubsystem.WheelDirection.OUTTAKE);
                ArmConstants.isHome = false;
            }
        }, armSubsystem, clawSubsystem));

        new JoystickButton(driverJoystick, 3).onTrue(Commands.runOnce(() -> {
            DriveConstants.slowMode = !DriveConstants.slowMode;
        }));

        new JoystickButton(driverJoystick, 2).onTrue(Commands.runOnce(() -> {
            armSubsystem.setSetpoint(Math.toRadians(-75));
            new Thread(() -> {
                try {
                    Thread.sleep(250);
                    clawSubsystem.setSetPoint(0);  
                } catch (InterruptedException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }).start();

            ArmConstants.isScore = false;
            ArmConstants.isHome = false;
            clawSubsystem.currWheelDirection = WheelDirection.INTAKE;
        }));

        new JoystickButton(driverJoystick, 4).onTrue(Commands.runOnce(() -> {
            DriveConstants.kFieldCentric = !DriveConstants.kFieldCentric;
        }));
    }

    public double[] getPositionsToScore() {
        if (clawSubsystem.getCurrGameElement() == GameElement.CONE) {
            if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.LOW) {
                return new double[] { Constants.ArmConstants.kArmAngleRadConeLow,
                    0 };
            } else if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.MID) {
                return new double[] { Constants.ArmConstants.kArmAngleRadConeMid,
                    ClawConstants.kClawAngleRadScoreCone };
            } else {
                return new double[] { Constants.ArmConstants.kArmAngleRadConeHigh,
                    ClawConstants.kClawAngleRadScoreCone };
            }
        } else {
            if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.LOW) {
                return new double[] { Constants.ArmConstants.kArmAngleRadCubeLow,
                    ClawConstants.kClawAngleRadScoreCube };
            }
            if (armSubsystem.getDesiredArmPosition() == ArmSubsystem.ArmPosition.MID) {
                return new double[] { Constants.ArmConstants.kArmAngleRadCubeMid,
                    ClawConstants.kClawAngleRadScoreCube };
            } else {
                return new double[] { Constants.ArmConstants.kArmAngleRadCubeHigh,
                    ClawConstants.kClawAngleRadScoreCube };
            }
        }
    }

    public Command getAutonomousCommand() {
            clawSubsystem.currWheelDirection = WheelDirection.INTAKE;
            clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);

            if(autoStartSideChooser.getSelected() == "BlueSide") {
                if(autoRoutineChooser.getSelected() == "AutoOpen") {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("OpenExitComunityPathBlue", new PathConstraints(4, 4));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else if (autoRoutineChooser.getSelected() == "AutoMid"){
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("MiddleExitComunityPathBlue", new PathConstraints(2, 2));
                    PathPlannerTrajectory trajStrafe = PathPlanner.loadPath("MiddleStrafePathBlue", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);
                    Command traj2 = swerveSubsystem.followTrajectoryCommand(trajStrafe, false);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        traj2,
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("CableCoverExitComunityPathBlue", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
            }
            else {
                if(autoRoutineChooser.getSelected() == "AutoOpen") {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("OpenExitComunityPathRed", new PathConstraints(4, 4));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else if(autoRoutineChooser.getSelected() == "AutoMid") {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("MiddleExitComunityPathRed", new PathConstraints(2, 2));
                    PathPlannerTrajectory trajStrafe = PathPlanner.loadPath("MiddleStrafePathRed", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);
                    Command traj2 = swerveSubsystem.followTrajectoryCommand(trajStrafe, false);


                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    clawSubsystem.periodic();
                                    armSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        traj2,
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
                else {
                    PathPlannerTrajectory trajExitComunity = PathPlanner.loadPath("CableCoverExitComunityRedPath", new PathConstraints(2, 2));

                    Command traj1 = swerveSubsystem.followTrajectoryCommand(trajExitComunity, true);

                    return new SequentialCommandGroup(
                        new SequentialCommandGroup(
                            Commands.runOnce(() -> {
                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadCubeHigh);
                                clawSubsystem.setSetPoint(ClawConstants.kClawAngleRadScoreCube);

                                double zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1.5) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                clawSubsystem.setWheelDirection(WheelDirection.OUTTAKE);

                                zeroTime = Timer.getFPGATimestamp();
                                while(Timer.getFPGATimestamp() - zeroTime <= 1) {
                                    armSubsystem.periodic();
                                    clawSubsystem.periodic();
                                }

                                armSubsystem.setSetpoint(ArmConstants.kArmAngleRadHome);
                                clawSubsystem.setSetPoint(ClawConstants.kAngleRadHome);
                                clawSubsystem.setWheelDirection(WheelDirection.OFF);
                            }, armSubsystem, clawSubsystem),
                            traj1
                        ),
                        Commands.runOnce(() -> swerveSubsystem.stopModules(), swerveSubsystem)
                    );
                }
        }
    }

    public boolean getConfigured() {
        return configured;
    }
}