package org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.kalipsorobotics.biobuzz.RedTippingAuto.Cell;
import org.firstinspires.ftc.teamcode.kalipsorobotics.math.Point;
import org.firstinspires.ftc.teamcode.kalipsorobotics.navigation.RobotFootprint;
import org.firstinspires.ftc.teamcode.kalipsorobotics.utilities.KLog;

/**
 * RedDecideAuto for when the PARTNER shoots first (solo preload, the partner robot).
 *
 * The partner starts at the A end, where Cell A is up, and shoots its 4 preload into it. That tips A, and Cell B stands up.
 * We start at the B end ("the down side"), WAIT there until Cell B is up, then shoot our preload into it.
 * Everything after that is exactly RedDecideAuto: it decides every move the same way.
 *
 * It only changes the start, and what we believe at the start: "A already tipped, B is up and empty".
 * The numbers marked GUESS are unmeasured.
 */
@Autonomous(name = "Red Decide Auto (partner first)", group = "biobuzz")
public class RedDecideAutoPartnerFirst extends RedDecideAuto {

    private static final double START_X_MM = 1500;                  // GUESS: far enough from Flower B (x 1155-1283) that our body does not touch it
    private static final double CELL_B_UP_AT_SECONDS = 4.7;         // GUESS: the Hive finishes tipping about 3 s after the partner's last ball (sim: last ball 1.65 s, up at about 4.65 s)

    @Override
    Point startPoint() {
        return new Point(START_X_MM, RobotFootprint.BACK_MM);       // the back touches the wall at y = 0
    }

    @Override
    double startHeadingRad() {
        return Math.toRadians(90);                                  // the front points up the field, toward the Hive
    }

    @Override
    Point firstTurretTarget() {
        return FieldConfig.bTargetPoint;                            // the first Cell WE shoot is B
    }

    /**
     * The partner is shooting into A right now. Believe the result: A tipped, B is up and empty.
     * The balls of the tipped A Cell spill on the A side, so A goes on the spill list.
     * Then do not move until we would arrive at the B shooting spot about when B is up. Shooting earlier sends the balls into the wrong Cell.
     */
    @Override
    void afterStart() {
        tipsDone = 1;
        upCell = Cell.B;
        ballsInCellA = 0;
        ballsInCellB = 0;
        pendingSpills.add(Cell.A);
        double secondsToDrive = driveSeconds(robotPoint(), launchPointOf(Cell.B));
        decideNotBeforeSeconds = Math.max(0, CELL_B_UP_AT_SECONDS - secondsToDrive);
        hiveSettledAtSeconds = CELL_B_UP_AT_SECONDS;   // the same wait, now enforced at the shot even if we drive faster
        KLog.d("Decide", () -> "partner first: B is up at " + CELL_B_UP_AT_SECONDS + " s, so we start driving at " + decideNotBeforeSeconds + " s");
    }
}
