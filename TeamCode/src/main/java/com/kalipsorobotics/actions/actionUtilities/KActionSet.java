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
        for (Action a : actions) {
            if (a != null && a.dependentActionsDone()) {
                KLog.d("action set log", "executing " + a);
                a.updateCheckDone();
            }
        }
    }


    @Override
    public boolean updateCheckDone(){
        if (isDone) {
            KLog.d("action set log", "done for " + name);
            return true;
        }

        update();
        return updateIsDone();
    }

    @Override
    protected boolean updateIsDone() {
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
