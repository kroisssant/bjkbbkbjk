package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class TriggerCommand extends CommandBase {
    IntakeSubsystem intakeSubsystem;
    DoubleSupplier leftTrigger;
    DoubleSupplier rightTrigger;

    public TriggerCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        if(rightTrigger.getAsDouble() > 0.1){
            intakeSubsystem.setIntakePower(-0.65);
        } else if(leftTrigger.getAsDouble() > 0.1){
            intakeSubsystem.setIntakePower(0.65);
        } else {
            intakeSubsystem.end();
        }
    }
}
