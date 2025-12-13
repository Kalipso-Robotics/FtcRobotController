package com.kalipsorobotics.actions.actionUtilities;

import com.kalipsorobotics.utilities.KLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;

public abstract class Action {

    protected ArrayList<Action> dependentActions = new ArrayList<>();
    protected boolean hasStarted = false;
    protected boolean isDone = false;

    protected String name;

    protected Telemetry telemetry;

    // Timing and logging fields
    private long startTimeMs = 0;
    private long lastUpdateTimeMs = 0;
    private long blockedStartTimeMs = 0;
    private boolean wasBlockedLastUpdate = false;
    private int updateCount = 0;

    public boolean getIsDone() {
        return isDone;
    }

    public void setIsDone(boolean isDone) {
        this.isDone = isDone;
    }

    public boolean getHasStarted() {
        return hasStarted;
    }

    public void setDependentActions(Action... actions) {
        if (dependentActions == null) {
            dependentActions = new ArrayList<>();
        }
        dependentActions.clear();
        Collections.addAll(dependentActions, actions);

    }

    public void setDependentActions(ArrayList<Action> actions) {
        if (dependentActions == null) {
            dependentActions = new ArrayList<>();
        }
        dependentActions.clear();
        for (Action a : actions) {
            dependentActions.add(a);
        }

    }

    ArrayList<Action> getDependentActions() {
        return this.dependentActions;
    }

    //updates the action
    public boolean updateCheckDone() {
        if (isDone) {
            return true;
        } //if done never update

        // Check if blocked by dependent actions
        boolean isBlocked = false;
        StringBuilder blockedBy = new StringBuilder();

        for (Action action : dependentActions) {
            if (action != null && !action.getIsDone()) {
                isBlocked = true;
                if (blockedBy.length() > 0) blockedBy.append(", ");
                blockedBy.append(action.getName() != null ? action.getName() : "unnamed");
            }
        }

        if (isBlocked) {
            // Track when blocking started
            if (!wasBlockedLastUpdate) {
                blockedStartTimeMs = System.currentTimeMillis();
                wasBlockedLastUpdate = true;
                KLog.d("ActionBlocking", String.format("[%s] BLOCKED - Waiting on: [%s]",
                    getName() != null ? getName() : "unnamed", blockedBy.toString()));
            } else {
                // Log periodic updates while blocked
                long blockedDurationMs = System.currentTimeMillis() - blockedStartTimeMs;
                if (blockedDurationMs > 0 && blockedDurationMs % 1000 < 50) { // Log every ~1 second
                    KLog.d("ActionBlocking", String.format("[%s] STILL BLOCKED for %.1fs - Waiting on: [%s]",
                        getName() != null ? getName() : "unnamed",
                        blockedDurationMs / 1000.0,
                        blockedBy.toString()));
                }
            }
            return false;
        }

        // No longer blocked
        if (wasBlockedLastUpdate) {
            long totalBlockedMs = System.currentTimeMillis() - blockedStartTimeMs;
            KLog.d("ActionBlocking", String.format("[%s] UNBLOCKED after %.1fs - Dependencies completed",
                getName() != null ? getName() : "unnamed", totalBlockedMs / 1000.0));
            wasBlockedLastUpdate = false;
        }

        // Track action start
        if (!hasStarted) {
            startTimeMs = System.currentTimeMillis();
            KLog.d("ActionLifecycle", String.format("[%s] STARTED", getName() != null ? getName() : "unnamed"));
        }

        update();
        updateCount++;
        lastUpdateTimeMs = System.currentTimeMillis();

        // Check if completed
        boolean justCompleted = !isDone && isUpdateDone();
        if (justCompleted) {
            long totalDurationMs = System.currentTimeMillis() - startTimeMs;
            KLog.d("ActionLifecycle", String.format("[%s] COMPLETED after %.1fs (%d updates)",
                getName() != null ? getName() : "unnamed",
                totalDurationMs / 1000.0,
                updateCount));
        }

        return isUpdateDone();

    }

    public boolean dependentActionsDone() {
        for (Action a : dependentActions) {
            if (a != null && !a.getIsDone()) {
                return false;
            }
        }
        return true;
    }

    protected boolean isUpdateDone() {
        return isDone;
    }

    //motor power, etc
    protected abstract void update();

    @Override
    public String toString() {
        return "Action {" +
                "name='" + name + '\'' + "status=" + isDone +
                '}';
    }


    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

//    public void setTelemetry(Telemetry telemetry) {
//        this.telemetry = telemetry;
//        telemetry.addData("action_"+this.getName(), 0);
//    }

    public void printWithDependentActions() {
        StringBuilder stringBuilder = new StringBuilder();
        stringBuilder.append(this.name);
        stringBuilder.append("--->");
        for (Action a : dependentActions) {
            if (a == null) {
                stringBuilder.append("null");
            } else {
                stringBuilder.append(a);
            }
        }
        KLog.d("action dependencies",  stringBuilder.toString());
    }
}
