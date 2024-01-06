package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.message.redux.ReceiveGamepadState;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Commands.ManualSlidesCommand;
import org.firstinspires.ftc.teamcode.Commands.TriggerCommand;
import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.GlisiereSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ScoringSubsystem;

import java.util.Timer;

@TeleOp
public class TeleOpConcurs extends CommandOpMode {

    DriveSubsystem driveSubsystem;
    GlisiereSubsystem glisiereSubsystem;
    IntakeSubsystem intakeSubsystem;
    ScoringSubsystem scoringSubsystem;
    DriveCommand driveCommand;
    GamepadEx driver1;
    GamepadEx driver2;

    private InstantCommand pressureOpen;
    private InstantCommand pressureClose;

    private InstantCommand pivot;
    private InstantCommand bratUp;

    private InstantCommand glisieraUp;
    private InstantCommand glisieraDown;

    private SequentialCommandGroup toScoreSequence1;
    private SequentialCommandGroup toScoreSequence2;

    private TriggerReader left_trigger_driver1;
    private TriggerReader left_trigger_driver2;
    private TriggerReader right_trigger_driver1;
    private TriggerReader right_trigger_driver2;


    private TriggerCommand intakeTriggerCommand;
    private ManualSlidesCommand manualSlidesCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        glisiereSubsystem = new GlisiereSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        scoringSubsystem = new ScoringSubsystem(hardwareMap);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        intakeTriggerCommand = new TriggerCommand(intakeSubsystem, ()-> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), ()-> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftY, driver1::getLeftX, driver1::getRightX);
        manualSlidesCommand = new ManualSlidesCommand(glisiereSubsystem, scoringSubsystem::wasPressureClosed, () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        left_trigger_driver1 = new TriggerReader(driver1, GamepadKeys.Trigger.LEFT_TRIGGER);
        right_trigger_driver1 = new TriggerReader(driver1, GamepadKeys.Trigger.RIGHT_TRIGGER);
        left_trigger_driver2 = new TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER);
        right_trigger_driver2 = new TriggerReader(driver2, GamepadKeys.Trigger.RIGHT_TRIGGER);

        pressureOpen = new InstantCommand(() -> {
            scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
            scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
            scoringSubsystem.pressureToggle = false;
        });

        pivot = new InstantCommand(() -> {
            if(scoringSubsystem.pivot.getPosition() == Constants.PIVOT_JOS){
                scoringSubsystem.setPivot(Constants.PIVOT_SUS);
            } else {
                scoringSubsystem.setPivot(Constants.PIVOT_JOS);
            }
        });

        pressureClose = new InstantCommand(() -> {
            scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
            scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
            scoringSubsystem.pressureToggle = true;
        });

        glisieraUp = new InstantCommand(() -> {
            glisiereSubsystem.glisiereAutoToggle = 2;
            glisiereSubsystem.setGlisiereFinalPosition(Constants.GLISIERA_UP);
        });

        glisieraDown = new InstantCommand(() -> {
            glisiereSubsystem.glisiereAutoToggle = 2;
            glisiereSubsystem.setGlisiereFinalPosition(Constants.GLISIERA_DOWN);
        });

        toScoreSequence1 = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    scoringSubsystem.setPivot(Constants.PIVOT_SUS_TELEOP);
                }),

                new WaitCommand(Constants.WAIT_FOR_PIVOT),

                new InstantCommand(()-> {
                    scoringSubsystem.setBratPos(Constants.BRAT_SUS);
                    glisiereSubsystem.glisiereAutoToggle = 2;
                    glisiereSubsystem.setGlisiereFinalPosition(Constants.GLISIERA_UP);
                })
        );

        toScoreSequence2 = new SequentialCommandGroup(
                new InstantCommand(()-> {
                    scoringSubsystem.setBratPos(Constants.BRAT_JOS);
                    scoringSubsystem.setPivot(Constants.PIVOT_JOS);
                }),

                new WaitCommand(Constants.WAIT_FOR_PIVOT_DOWN),

                new InstantCommand(() -> {
                    glisiereSubsystem.glisiereAutoToggle = 2;
                    glisiereSubsystem.setGlisiereFinalPosition(Constants.GLISIERA_DOWN);
                })

        );

        //nou
//        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whileHeld(new InstantCommand(intakeSubsystem::runFwd))
//                .whenReleased(new InstantCommand(intakeSubsystem::end));
//
//        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whileHeld(new InstantCommand(intakeSubsystem::runRvs))
//                .whenReleased(new InstantCommand(intakeSubsystem::end));

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(pivot);

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(toScoreSequence1);

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(toScoreSequence2);

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new InstantCommand(()-> {
                    glisiereSubsystem.delta = 1;
                    glisiereSubsystem.glisiereAutoToggle = 1;

                        })
                )
                .whenReleased(new InstantCommand(()-> {
                    glisiereSubsystem.glisiereAutoToggle = 0;
                }));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new InstantCommand(()-> {
                    glisiereSubsystem.delta = -1;
                    glisiereSubsystem.glisiereAutoToggle = 1;
                }))
                .whenReleased(new InstantCommand(()-> {
                    glisiereSubsystem.glisiereAutoToggle = 0;
                }));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(()-> {
                            scoringSubsystem.setBratPos(scoringSubsystem.bratDreapta.getPosition()+0.1);
                        })
                );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(()-> {
                    scoringSubsystem.setBratPos(scoringSubsystem.bratDreapta.getPosition()-0.1);
                        })
                );

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(
                        new ConditionalCommand(
                                pressureOpen,
                                pressureClose,
                                () -> scoringSubsystem.pressureToggle
                        )
                );

        driver1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(
                        new ConditionalCommand(
                                new InstantCommand(()->{
                                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_DESCHIS);
                                    scoringSubsystem.pressureStangaToggle = false;
                                }),
                                new InstantCommand(()->{
                                    scoringSubsystem.setPressureStangaPos(Constants.PRESSURE_STANGA_INCHIS);
                                    scoringSubsystem.pressureStangaToggle = true;

                                }),
                                () -> scoringSubsystem.pressureStangaToggle

                        )
                );

        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new ConditionalCommand(
                                new InstantCommand(()->{
                                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_DESCHIS);
                                    scoringSubsystem.pressureDreaptaToggle = false;
                                }),
                                new InstantCommand(()->{
                                    scoringSubsystem.setPressureDreaptaPos(Constants.PRESSURE_DREAPTA_INCHIS);
                                    scoringSubsystem.pressureDreaptaToggle = true;

                                }),
                                () -> scoringSubsystem.pressureDreaptaToggle
                        )
                );

        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new ConditionalCommand(
                                new InstantCommand(()->intakeSubsystem.dropdownUp()),
                                new InstantCommand(()->intakeSubsystem.dropdownDown()),
                                intakeSubsystem::isDropDownDown
                        )
                );

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whileHeld(new InstantCommand(()->driveCommand.slowed = true))
                        .whenReleased(new InstantCommand(()->driveCommand.slowed = false));

        driveSubsystem.setDefaultCommand(driveCommand);
        glisiereSubsystem.setDefaultCommand(manualSlidesCommand);
        intakeSubsystem.setDefaultCommand(intakeTriggerCommand);

        register(glisiereSubsystem);
    }
}