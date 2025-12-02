package com.kalipsorobotics.utilities;

import com.kalipsorobotics.utilities.KLog;

import com.qualcomm.robotcore.hardware.Gamepad;

public class KGamePad {

    private final Gamepad gamepad;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousButtonB = false;
    private boolean previousButtonX = false;
    private boolean previousButtonA = false;
    private boolean previousButtonY = false;
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;
    private boolean previousBackButton = false;
    private boolean previousStartButton = false;
    private boolean previousLeftTrigger = false;
    private boolean previousRightTrigger = false;
    private boolean previousLeftStickButton = false;
    private boolean previousRightStickButton = false;

    private boolean toggleY = false;
    private boolean toggleX = false;


    public KGamePad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public Gamepad getGamepad (){
        return gamepad;
    }

    public boolean isLeftBumperPressed(){
        return gamepad.left_bumper;
    }

    public boolean isRightBumperPressed(){
        return gamepad.right_bumper;
    }

    public boolean isDpadLeftFirstPressed(){
        boolean current = gamepad.dpad_left;
        boolean toggle = false;
        if (!this.previousDpadLeft && current){
            toggle = true;
            KLog.d("KGamePad", "This Dpad left toggle");
        }

        this.previousDpadLeft = current;
        return toggle;
    }

    public boolean isDpadLeftPressed() {
        return gamepad.dpad_left;
    }

    public boolean isDpadRightPressed() {
        return gamepad.dpad_right;
    }

    public boolean isDpadRightFirstPressed(){
        boolean current = gamepad.dpad_right;
        boolean toggle = false;
        if (!this.previousDpadRight && current){
            toggle = true;
            KLog.d("KGamePad", "This Dpad right toggle");
        }
        this.previousDpadRight = current;
        return toggle;
    }

    public boolean isDpadUpFirstPressed(){
        boolean current = gamepad.dpad_up;
        boolean toggle = false;
        if (!this.previousDpadUp && current){
            toggle = true;
            KLog.d("KGamePad", "This Dpad up toggle");
        }
        this.previousDpadUp = current;
        return toggle;
    }

    public boolean isDpadDownFirstPressed(){
        boolean current = gamepad.dpad_down;
        boolean toggle = false;
        if (!this.previousDpadDown && current){
            toggle = true;
            KLog.d("KGamePad", "This Dpad down toggle");
        }
        this.previousDpadDown = current;
        return toggle;
    }

    public boolean isButtonBFirstPressed(){
        boolean current = gamepad.b;
        boolean toggle = false;
        if(!this.previousButtonB && current){
            toggle = true;
            KLog.d("KGamePad", "This button B toggle");
        }
        this.previousButtonB = current;
        return toggle;
    }

    public boolean isButtonXFirstPressed(){
        boolean current = gamepad.x;
        boolean toggle = false;
        if(!this.previousButtonX && current){
            toggle = true;
            KLog.d("KGamePad", "This button X toggle");
        }
        this.previousButtonX = current;
        return toggle;
    }

    public boolean isButtonAFirstPressed(){
        boolean current = gamepad.a;
        boolean toggle = false;
        if(!this.previousButtonA && current){
            toggle = true;
            KLog.d("KGamePad", "This button A toggle");
        }
        this.previousButtonA = current;
        return toggle;
    }

    public boolean isButtonYFirstPressed(){
        boolean current = gamepad.y;
        boolean toggle = false;
        if(!this.previousButtonY && current){
            toggle = true;
            KLog.d("KGamePad", "This button Y toggle");
        }
        this.previousButtonY = current;
        return toggle;
    }

    public boolean isYPressed() {
        return gamepad.y;
    }

    public boolean isXPressed() {
        return gamepad.x;
    }

    public boolean isToggleX() {
        if (isButtonXFirstPressed()) {
            toggleX = !toggleX;
        }
        KLog.d("KGamePad_Toggle", "Toggle X: " + toggleX);
        return toggleX;
    }

    public boolean isToggleY() {
        if (isButtonYFirstPressed()) {
            toggleY = !toggleY;
        }
        return toggleY;
    }

    public boolean isRightBumperFirstPressed(){
        boolean current = gamepad.right_bumper;
        boolean toggle = false;
        if(!this.previousRightBumper && current){
            toggle = true;
            KLog.d("KGamePad", "This right bumper toggle");
        }
        this.previousRightBumper = current;
        return toggle;
    }

    public boolean isLeftBumperFirstPressed(){
        boolean current = gamepad.left_bumper;
        boolean toggle = false;
        if(!this.previousLeftBumper && current){
            toggle = true;
            KLog.d("KGamePad", "This left bumper toggle");
        }
        this.previousLeftBumper = current;
        return toggle;
    }

    public boolean isRightTriggerPressed(){
        return gamepad.right_trigger > 0.5;
    }

    public boolean isLeftTriggerPressed(){
        return gamepad.left_trigger > 0.5;
    }

    public boolean isLeftTriggerFirstPressed(){
        boolean current = gamepad.left_trigger > 0.5;
        boolean toggle = false;
        if(!this.previousLeftTrigger && current){
            toggle = true;
            KLog.d("KGamePad", "This left trigger toggle");
        }
        this.previousLeftTrigger = current;
        return toggle;
    }

    public boolean isRightTriggerFirstPressed(){
        boolean current = gamepad.right_trigger > 0.5;
        boolean toggle = false;
        if(!this.previousRightTrigger && current){
            toggle = true;
            KLog.d("KGamePad", "This right trigger toggle");
        }
        this.previousRightTrigger = current;
        return toggle;
    }

    public boolean isLeftStickButtonFirstPressed(){
        boolean current = gamepad.left_stick_button;
        boolean toggle = false;
        if(!this.previousLeftStickButton && current){
            toggle = true;
            KLog.d("KGamePad", "This left stick button toggle");
        }
        this.previousLeftStickButton = current;
        return toggle;
    }

    public boolean isRightStickButtonFirstPressed(){
        boolean current = gamepad.right_stick_button;
        boolean toggle = false;
        if(!this.previousRightStickButton && current){
            toggle = true;
            KLog.d("KGamePad", "This right stick button toggle");
        }
        this.previousRightStickButton = current;
        return toggle;
    }

    public boolean isBackButtonPressed(){
        boolean current = gamepad.back;
        boolean toggle = false;
        if(!this.previousBackButton && current){
            toggle = true;
            KLog.d("KGamePad", "This back button toggle");
        }
        this.previousBackButton = current;
        return toggle;
    }

    public boolean isStartButtonPressed(){
        boolean current = gamepad.start;
        boolean toggle = false;
        if(!this.previousStartButton && current){
            toggle = true;
            KLog.d("KGamePad", "This start button toggle");
        }
        this.previousStartButton = current;
        return toggle;
    }

    public double getLeftStickX(){
        return gamepad.left_stick_x;
    }

    public double getLeftStickY(){
        return gamepad.left_stick_y;
    }

    public double getRightStickX(){
        return gamepad.right_stick_x;
    }

    public double getRightStickY(){
        return gamepad.right_stick_y;
    }



}
