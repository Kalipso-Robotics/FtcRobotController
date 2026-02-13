package org.firstinspires.ftc.teamcode.kalipsorobotics.utilities;

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

    private boolean toggleA = false;
    private boolean toggleB = false;
    private boolean toggleX = false;
    private boolean toggleY = false;
    private boolean toggleBack = false;
    private boolean toggleStart = false;
    private boolean toggleDpadUp = false;
    private boolean toggleDpadDown = false;
    private boolean toggleDpadLeft = false;
    private boolean toggleDpadRight = false;
    private boolean toggleLeftBumper = false;
    private boolean toggleRightBumper = false;
    private boolean toggleLeftTrigger = false;
    private boolean toggleRightTrigger = false;
    private boolean toggleLeftStickButton = false;
    private boolean toggleRightStickButton = false;


    public KGamePad(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public Gamepad getGamepad (){
        return gamepad;
    }

    public boolean isLeftBumperPressed(){
        KLog.d("KGamePad", "Left Bumper: " + gamepad.left_bumper);
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

    public boolean isDpadUpPressed() {
        return gamepad.dpad_up;
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

    public boolean isDpadDownPressed() {
        return gamepad.dpad_down;
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

    public boolean isBPressed() {
        return gamepad.b;
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

    public boolean isAPressed() {
        return gamepad.a;
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

    public void setToggleX(boolean toggleX) {
        this.toggleX = toggleX;
    }

    public void setToggleY(boolean toggleY) {
        this.toggleY = toggleY;
    }

    public void setToggleA(boolean toggleA) {
        this.toggleA = toggleA;
    }

    public void setToggleB(boolean toggleB) {
        this.toggleB = toggleB;
    }

    public void setToggleBack(boolean toggleBack) {
        this.toggleBack = toggleBack;
    }

    public void setToggleStart(boolean toggleStart) {
        this.toggleStart = toggleStart;
    }

    public void setToggleDpadUp(boolean toggleDpadUp) {
        this.toggleDpadUp = toggleDpadUp;
    }

    public void setToggleDpadDown(boolean toggleDpadDown) {
        this.toggleDpadDown = toggleDpadDown;
    }

    public void setToggleDpadLeft(boolean toggleDpadLeft) {
        this.toggleDpadLeft = toggleDpadLeft;
    }

    public void setToggleDpadRight(boolean toggleDpadRight) {
        this.toggleDpadRight = toggleDpadRight;
    }

    public void setToggleLeftBumper(boolean toggleLeftBumper) {
        this.toggleLeftBumper = toggleLeftBumper;
    }

    public void setToggleRightBumper(boolean toggleRightBumper) {
        this.toggleRightBumper = toggleRightBumper;
    }

    public void setToggleLeftTrigger(boolean toggleLeftTrigger) {
        this.toggleLeftTrigger = toggleLeftTrigger;
    }

    public void setToggleRightTrigger(boolean toggleRightTrigger) {
        this.toggleRightTrigger = toggleRightTrigger;
    }

    public void setToggleLeftStickButton(boolean toggleLeftStickButton) {
        this.toggleLeftStickButton = toggleLeftStickButton;
    }

    public void setToggleRightStickButton(boolean toggleRightStickButton) {
        this.toggleRightStickButton = toggleRightStickButton;
    }

    public boolean isToggleY() {
        if (isButtonYFirstPressed()) {
            toggleY = !toggleY;
        }
        return toggleY;
    }

    public boolean isToggleA() {
        if (isButtonAFirstPressed()) {
            toggleA = !toggleA;
        }
        return toggleA;
    }

    public boolean isToggleB() {
        if (isButtonBFirstPressed()) {
            toggleB = !toggleB;
        }
        return toggleB;
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

    public boolean isLeftStickButtonPressed() {
        return gamepad.left_stick_button;
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

    public boolean isRightStickButtonPressed() {
        return gamepad.right_stick_button;
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

    public boolean isBackButtonPressed() {
        return gamepad.back;
    }

    public boolean isBackButtonFirstPressed(){
        boolean current = gamepad.back;
        boolean toggle = false;
        if(!this.previousBackButton && current){
            toggle = true;
            KLog.d("KGamePad", "This back button toggle");
        }
        this.previousBackButton = current;
        return toggle;
    }

    public boolean isBackButtonToggle() {
        if (isBackButtonFirstPressed()) {
            toggleBack = !toggleBack;
        }
        KLog.d("KGamePad_Toggle", "Toggle Back: " + toggleBack);
        return toggleBack;
    }

    public boolean isStartButtonPressed() {
        return gamepad.start;
    }

    public boolean isStartButtonFirstPressed(){
        boolean current = gamepad.start;
        boolean toggle = false;
        if(!this.previousStartButton && current){
            toggle = true;
            KLog.d("KGamePad", "This start button toggle");
        }
        this.previousStartButton = current;
        return toggle;
    }

    public boolean isToggleStart() {
        if (isStartButtonFirstPressed()) {
            toggleStart = !toggleStart;
        }
        return toggleStart;
    }

    public boolean isToggleDpadUp() {
        if (isDpadUpFirstPressed()) {
            toggleDpadUp = !toggleDpadUp;
        }
        return toggleDpadUp;
    }

    public boolean isToggleDpadDown() {
        if (isDpadDownFirstPressed()) {
            toggleDpadDown = !toggleDpadDown;
        }
        return toggleDpadDown;
    }

    public boolean isToggleDpadLeft() {
        if (isDpadLeftFirstPressed()) {
            toggleDpadLeft = !toggleDpadLeft;
        }
        return toggleDpadLeft;
    }

    public boolean isToggleDpadRight() {
        if (isDpadRightFirstPressed()) {
            toggleDpadRight = !toggleDpadRight;
        }
        return toggleDpadRight;
    }

    public boolean isToggleLeftBumper() {
        if (isLeftBumperFirstPressed()) {
            toggleLeftBumper = !toggleLeftBumper;
        }
        return toggleLeftBumper;
    }

    public boolean isToggleRightBumper() {
        if (isRightBumperFirstPressed()) {
            toggleRightBumper = !toggleRightBumper;
        }
        return toggleRightBumper;
    }

    public boolean isToggleLeftTrigger() {
        if (isLeftTriggerFirstPressed()) {
            toggleLeftTrigger = !toggleLeftTrigger;
        }
        return toggleLeftTrigger;
    }

    public boolean isToggleRightTrigger() {
        if (isRightTriggerFirstPressed()) {
            toggleRightTrigger = !toggleRightTrigger;
        }
        return toggleRightTrigger;
    }

    public boolean isToggleLeftStickButton() {
        if (isLeftStickButtonFirstPressed()) {
            toggleLeftStickButton = !toggleLeftStickButton;
        }
        return toggleLeftStickButton;
    }

    public boolean isToggleRightStickButton() {
        if (isRightStickButtonFirstPressed()) {
            toggleRightStickButton = !toggleRightStickButton;
        }
        return toggleRightStickButton;
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

    public boolean isLeftStickActive() {
        return gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0;
    }

    public boolean isRightStickActive() {
        return gamepad.right_stick_x != 0 || gamepad.right_stick_y != 0;
    }

    public boolean isAnyStickActive() {
        return isLeftStickActive() || isRightStickActive();
    }

}
