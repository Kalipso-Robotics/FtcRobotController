package com.kalipsorobotics.actions.actionUtilities;

import com.kalipsorobotics.utilities.KLog;

import java.util.ArrayList;
import java.util.Collections;

public class KActionSet extends Action {

    ArrayList<Action> actions = new ArrayList<>();

    public void addAction(Action... actions) {
        Collections.addAll(this.actions, actions);
    }

    @Override
    public void update() {
        beforeUpdate();

        int totalActions = 0;
        int executedActions = 0;
        int completedActions = 0;
        int blockedActions = 0;

        for (Action a : actions) {
            if (a != null) {
                totalActions++;
                if (a.getIsDone()) {
                    completedActions++;
                } else if (a.dependentActionsDone()) {
                    KLog.d("ActionSet", String.format("[%s] Executing action: [%s]",
                        getName() != null ? getName() : "unnamed",
                        a.getName() != null ? a.getName() : "unnamed"));
                    a.updateCheckDone();
                    executedActions++;
                } else {
                    blockedActions++;
                    // Find which dependencies are blocking
                    StringBuilder blocking = new StringBuilder();
                    for (Action dep : a.getDependentActions()) {
                        if (dep != null && !dep.getIsDone()) {
                            if (blocking.length() > 0) blocking.append(", ");
                            blocking.append(dep.getName() != null ? dep.getName() : "unnamed");
                        }
                    }
                    KLog.d("ActionSet", String.format("[%s] Skipping action [%s] - blocked by: [%s]",
                        getName() != null ? getName() : "unnamed",
                        a.getName() != null ? a.getName() : "unnamed",
                        blocking.toString()));
                }
            }
        }
        afterUpdate();
        KLog.d("ActionSet", String.format("[%s] Status: %d total, %d completed, %d executed, %d blocked",
            getName() != null ? getName() : "unnamed",
            totalActions, completedActions, executedActions, blockedActions));
    }

    // this is a hook to open up update to other code
    protected void beforeUpdate() {

    }

    protected void afterUpdate() {

    }

    @Override
    public boolean updateCheckDone() {
        if (isDone) {
            KLog.d("action set log", "done for " + name);
            return true;
        }

        update();
        return isUpdateDone();
    }

    @Override
    protected boolean isUpdateDone() {
        if(isDone) {
            return isDone;
        }
        for (Action a : actions) {
            if (a != null && !a.getIsDone()) {
                isDone = false;
                return isDone;
            }
        }
        isDone = true;
        return isDone;
    }

    public void printWithDependentActions() {
        KLog.d("action dependencies", "Start Action Set");
        super.printWithDependentActions();

        for (Action a : actions) {
            if (a != null) {
                a.printWithDependentActions();
            }
        }
        KLog.d("action dependencies", "End Action Set");

    }

    public void clear() {
        actions.clear();
    }

}
