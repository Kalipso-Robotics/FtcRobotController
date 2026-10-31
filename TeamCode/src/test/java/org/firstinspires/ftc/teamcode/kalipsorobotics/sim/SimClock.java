package org.firstinspires.ftc.teamcode.kalipsorobotics.sim;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.OpModeUtilities;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Semaphore;

/**
 * The simulator's clock. While a sim runs, every time call in the robot code (System.nanoTime,
 * Thread.sleep, SystemClock.elapsedRealtime, ...) is redirected here by ClockRewriter, so the robot
 * code sees VIRTUAL time instead of the laptop's clock.
 *
 * Virtual time only moves when every robot thread is waiting. Only ONE robot thread runs at a time
 * (it holds the "baton"), so a run gives the same result every time and can go faster than real time.
 * When all threads are waiting, time jumps to the earliest wake-up and the physics is told to catch up.
 *
 * A thread waits in three ways:
 *   sleep(ms)      : wakes after that many virtual milliseconds.
 *   loopBoundary() : the thread finished one loop. It wakes after its loop period. This is the call
 *                    OpModeUtilities.clearBulkCache(), which the code already makes once per loop.
 *   yield()        : a tiny wait, only so that a thread that busy-waits still lets time move.
 *
 * A thread the code starts itself (the odometry thread) is given its slot when it is CREATED, by the
 * thread that creates it, so the order never depends on the laptop's real thread timing.
 *
 * When no sim is running, every method passes straight through to the real clock.
 */
public final class SimClock {

    /** The physics. Called (with all robot threads stopped) every time virtual time moves forward. */
    public interface TimeListener {
        void advanceTo(long toNanos);
    }

    /** One robot thread that the scheduler controls. */
    public static final class Participant {
        private final String name;
        private final long periodNanos;
        private final Semaphore baton = new Semaphore(0);
        private volatile Thread thread;
        private long wakeNanos;
        private boolean waiting;
        private boolean dead;

        private Participant(String name, long periodNanos) {
            this.name = name;
            this.periodNanos = periodNanos;
        }

        /** Call first thing on the thread this participant stands for. */
        public void bindToCurrentThread() {
            thread = Thread.currentThread();
            currentParticipant.set(this);
            baton.acquireUninterruptibly(); // wait for the first turn
        }
    }

    private static final long BOOT_NANOS = 5_000L * 1_000_000_000L; // "uptime" at virtual time 0
    private static final Object LOCK = new Object();
    private static final List<Participant> participants = new ArrayList<>();
    private static final ThreadLocal<Participant> currentParticipant = new ThreadLocal<>();

    private static volatile boolean active;
    private static volatile long nowNanos;
    private static long startWallMs;
    private static TimeListener listener;
    private static Participant running;
    private static long backgroundLoopNanos;
    private static long yieldNanos;
    private static int backgroundThreadCount;

    // A thread that polls opModeIsActive() before START is waiting for START, not working.
    private static final long WAITING_FOR_START_POLL_NANOS = 1_000_000L;

    private SimClock() {
    }

    // ---------------------------------------------------------------------------------------------
    // Run control (used by SimRunner)
    // ---------------------------------------------------------------------------------------------

    public static void start(TimeListener timeListener, long backgroundLoopPeriodNanos, long yieldCostNanos) {
        synchronized (LOCK) {
            participants.clear();
            running = null;
            nowNanos = 0;
            startWallMs = System.currentTimeMillis();
            listener = timeListener;
            backgroundLoopNanos = backgroundLoopPeriodNanos;
            yieldNanos = yieldCostNanos;
            backgroundThreadCount = 0;
            active = true;
        }
    }

    /** Creates a thread slot. Create them in a fixed order: that order breaks ties between threads. */
    public static Participant create(String name, long loopPeriodNanos) {
        synchronized (LOCK) {
            Participant p = new Participant(name, loopPeriodNanos);
            p.waiting = true;
            p.wakeNanos = nowNanos;
            participants.add(p);
            return p;
        }
    }

    /** Lets the first thread run. */
    public static void go() {
        synchronized (LOCK) {
            scheduleNextLocked();
        }
        Thread reaper = new Thread(SimClock::reapDeadHolders, "sim-clock-reaper");
        reaper.setDaemon(true);
        reaper.start();
    }

    /** The calling thread is done. Hands the baton on. */
    public static void threadFinished() {
        synchronized (LOCK) {
            Participant me = currentParticipant.get();
            if (me == null) {
                return;
            }
            me.dead = true;
            me.waiting = false;
            if (running == me) {
                running = null;
                scheduleNextLocked();
            }
        }
    }

    /** Ends the run: every waiting thread is let go so none stays stuck. */
    public static void stop() {
        synchronized (LOCK) {
            active = false;
            for (Participant p : participants) {
                p.dead = true;
                p.waiting = false;
                p.baton.release();
            }
            participants.clear();
            running = null;
        }
    }

    public static boolean isActive() {
        return active;
    }

    public static long elapsedNanos() {
        return nowNanos;
    }

    // ---------------------------------------------------------------------------------------------
    // Calls that ClockRewriter puts in place of the real time calls
    // ---------------------------------------------------------------------------------------------

    public static long nanoTime() {
        return active ? BOOT_NANOS + nowNanos : System.nanoTime();
    }

    public static long currentTimeMillis() {
        return active ? startWallMs + nowNanos / 1_000_000L : System.currentTimeMillis();
    }

    public static long elapsedRealtime() {
        return active ? (BOOT_NANOS + nowNanos) / 1_000_000L : SystemClock.elapsedRealtime();
    }

    public static long elapsedRealtimeNanos() {
        return active ? BOOT_NANOS + nowNanos : SystemClock.elapsedRealtimeNanos();
    }

    public static long uptimeMillis() {
        return active ? (BOOT_NANOS + nowNanos) / 1_000_000L : SystemClock.uptimeMillis();
    }

    public static void sleep(long millis) throws InterruptedException {
        if (!active) {
            Thread.sleep(millis);
            return;
        }
        block(self(), nowNanos + millis * 1_000_000L);
    }

    /** android.os.SystemClock.sleep: same as sleep, but does not throw. */
    public static void systemClockSleep(long millis) {
        try {
            sleep(millis);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static void yield() {
        if (!active) {
            Thread.yield();
            return;
        }
        try {
            block(self(), nowNanos + yieldNanos);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Replaces LinearOpMode.opModeIsActive(). While the OpMode is active the real method already yields.
     * Before START (or after STOP) a polling loop would never give up the baton, so it waits a little.
     */
    public static boolean opModeIsActive(Object opMode) {
        boolean isActive = ((LinearOpMode) opMode).opModeIsActive();
        if (!isActive && active) {
            try {
                block(self(), nowNanos + WAITING_FOR_START_POLL_NANOS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        return isActive;
    }

    /** Replaces Executors.newSingleThreadExecutor(): its thread joins the scheduler as it is created. */
    public static ExecutorService newSingleThreadExecutor() {
        if (!active) {
            return Executors.newSingleThreadExecutor();
        }
        return Executors.newSingleThreadExecutor(task -> {
            // runs on the thread that submits the first task, so the slot order is the same every run
            Participant slot = create("background-" + (++backgroundThreadCount), backgroundLoopNanos);
            return new Thread(() -> {
                slot.bindToCurrentThread();
                try {
                    task.run();
                } finally {
                    threadFinished();
                }
            }, slot.name);
        });
    }

    /** Replaces OpModeUtilities.clearBulkCache(): one call = one finished loop of this thread. */
    public static void loopBoundary(Object opModeUtilities) {
        if (!active) {
            ((OpModeUtilities) opModeUtilities).clearBulkCache();
            return;
        }
        Participant me = self();
        try {
            block(me, nowNanos + me.periodNanos);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Scheduler
    // ---------------------------------------------------------------------------------------------

    /** The calling thread's slot. A thread the code started itself (the odometry thread) joins on its first call. */
    private static Participant self() {
        Participant me = currentParticipant.get();
        if (me != null) {
            return me;
        }
        me = new Participant("background-" + Thread.currentThread().getName(), backgroundLoopNanos);
        me.thread = Thread.currentThread();
        currentParticipant.set(me);
        synchronized (LOCK) {
            participants.add(me);
            if (running == null) {
                running = me;
                return me;
            }
            me.waiting = true;
            me.wakeNanos = nowNanos;
        }
        me.baton.acquireUninterruptibly(); // wait for the first turn
        return me;
    }

    private static void block(Participant me, long wakeNanos) throws InterruptedException {
        synchronized (LOCK) {
            if (me.dead) {
                throw new InterruptedException("sim thread was stopped: " + me.name);
            }
            me.wakeNanos = wakeNanos;
            me.waiting = true;
            if (running == me) {
                running = null;
            }
            scheduleNextLocked();
        }
        try {
            me.baton.acquire();
        } catch (InterruptedException e) {
            synchronized (LOCK) {
                if (me.waiting) {
                    me.waiting = false; // it was not its turn: leave the scheduler
                    me.dead = true;
                } else {
                    me.baton.tryAcquire(); // it was already chosen: it keeps the baton
                }
            }
            throw e;
        }
    }

    /** Picks the thread that wakes first (ties: creation order), moves time to its wake-up, and gives it the baton. */
    private static void scheduleNextLocked() {
        Participant next = null;
        for (Participant p : participants) {
            if (p.waiting && (next == null || p.wakeNanos < next.wakeNanos)) {
                next = p;
            }
        }
        if (next == null) {
            running = null;
            return;
        }
        if (next.wakeNanos > nowNanos) {
            nowNanos = next.wakeNanos;
            if (listener != null) {
                listener.advanceTo(nowNanos);
            }
        }
        next.waiting = false;
        running = next;
        next.baton.release();
    }

    /** A thread that ends while holding the baton cannot hand it on, so this checks now and then. */
    private static void reapDeadHolders() {
        while (active) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                return;
            }
            synchronized (LOCK) {
                if (running != null && running.thread != null && !running.thread.isAlive()) {
                    running.dead = true;
                    running = null;
                    scheduleNextLocked();
                }
            }
        }
    }
}
