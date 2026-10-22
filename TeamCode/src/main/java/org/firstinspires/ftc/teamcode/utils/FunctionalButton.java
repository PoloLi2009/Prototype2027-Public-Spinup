package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.command.button.Button;

import java.util.function.BooleanSupplier;

/** FTCLib button backed by a lambda, matching the DECODE project's binding style. */
public class FunctionalButton extends Button {
    private final BooleanSupplier supplier;

    public FunctionalButton(BooleanSupplier supplier) {
        this.supplier = supplier;
    }

    @Override
    public boolean get() {
        return supplier.getAsBoolean();
    }
}
