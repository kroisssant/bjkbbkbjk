package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.GlisiereSubsystem;

import java.util.function.DoubleSupplier;

public class ManualSlidesCommand extends CommandBase {
    GlisiereSubsystem glisiereSubsystem;
    DoubleSupplier leftTrigger;
    DoubleSupplier rightTrigger;

    public ManualSlidesCommand(GlisiereSubsystem glisiereSubsystem, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        this.glisiereSubsystem = glisiereSubsystem;
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;

        addRequirements(glisiereSubsystem);
    }

    @Override
    public void execute() {
        if(Constants.slideInputState == Constants.SlideInputState.MANUAL) {
            //rightTrigger = mers in sus (blocheaza sinele daca merge mai sus decat trebuie
            double rightTriggerPower = glisiereSubsystem.getAveragePosition() > Constants.GLISIERA_UP ? glisiereSubsystem.getPassivePower() : Math.max(glisiereSubsystem.getPassivePower(), rightTrigger.getAsDouble());
            //leftTrigger = mers in jos (putere pasiva inmultita cu leftTrigger pentru o coborare inceata si lina)
            double leftTriggerPower = leftTrigger.getAsDouble() * (glisiereSubsystem.getPassivePower() + 0.05);

            double motorPower = rightTriggerPower - leftTriggerPower;
            glisiereSubsystem.setGlisieraManual(motorPower);
        }
    }
}
