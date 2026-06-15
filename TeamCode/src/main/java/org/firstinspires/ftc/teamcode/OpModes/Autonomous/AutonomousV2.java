package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutonomousV2 extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;


    // Estados
    public enum PathState{
        // START POS -> FINAL POS
        START_POS_TO_SHOOT,
        SHOOT_PRELOAD,
        SHOOT_TO_FIRST_COLLECT,

        FIRST_COLLECT_TO_OPEN_GATE,
        OPEN_GATE_TO_SHOOT,

        SHOOT_TO_SECOND_COLLECT,

        SECOND_COLLECT_TO_SHOOT,

        SHOOT_TO_OPEN_AND_COLLECT,

        SHOOT_TO_OPEN_AND_COLLECT_AND_OPEN_GATE,

        OPEN_AND_COLLECT_TO_SHOOT,

        OPEN_AND_COLLECT_TO_OPEN_GATE;

    }



    PathState pathState;

    // Posições

    private final Pose start = new Pose(111.61279069767443, 133.21104651162793, Math.toRadians(90));

    private final Pose shoot = new Pose(82.49011627906977, 81.90523255813953, Math.toRadians(45));

    private final Pose firstCollect = new Pose(123.2653648322401, 58.2868597168621, Math.toRadians(0));

    private final Pose openGate = new Pose(126.4042828838878, 63.812361437801925, Math.toRadians(0));

    private final Pose secondCollect = new Pose(124.37667052964221, 81.90523255813953, Math.toRadians(0));

    private final Pose openAndCollect = new Pose(131.17298164247407, 59.53094686461575, Math.toRadians(40));



    // Control Points
    private final Pose collectAndOpenControlPoint = new Pose(80.44983357891073, 51.30069730029153);
    private final Pose openAndCollectControlPoint = new Pose(102.90829314681841, 54.87157808347065);



    private PathChain
            driveStartToShoot,
            driveShootToFirstCollect,
            driveFirstCollectToOpenGate,
            driveOpenGateToShoot,
            driveShootToSecondCollect,
            driveSecondCollectToShoot,
            driveShootToOpenAndCollect,
            driveOpenAndCollectToShoot,
            driveOpenAndCollectToOpenGate;



    public void buildPaths(){
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(start, shoot))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();

        driveShootToFirstCollect = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, collectAndOpenControlPoint, firstCollect))
                .setLinearHeadingInterpolation(shoot.getHeading(), firstCollect.getHeading())
                .build();

        driveFirstCollectToOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(firstCollect, openGate))
                .setLinearHeadingInterpolation(firstCollect.getHeading(), openGate.getHeading())
                .build();

        driveOpenGateToShoot = follower.pathBuilder()
                .addPath(new BezierLine(openGate, shoot))
                .setLinearHeadingInterpolation(openGate.getHeading(), shoot.getHeading())
                .build();

        driveShootToSecondCollect = follower.pathBuilder()
                .addPath(new BezierLine(shoot, secondCollect))
                .setLinearHeadingInterpolation(shoot.getHeading(), secondCollect.getHeading())
                .build();

        driveSecondCollectToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondCollect, shoot))
                .setLinearHeadingInterpolation(secondCollect.getHeading(), shoot.getHeading())
                .build();

        driveShootToOpenAndCollect = follower.pathBuilder()
                .addPath(new BezierCurve(shoot, openAndCollectControlPoint, openAndCollect))
                .setLinearHeadingInterpolation(shoot.getHeading(), openAndCollect.getHeading())
                .build();

        driveOpenAndCollectToShoot = follower.pathBuilder()
                .addPath(new BezierLine(openAndCollect, shoot))
                .setLinearHeadingInterpolation(openAndCollect.getHeading(), shoot.getHeading())
                .build();

        driveOpenAndCollectToOpenGate = follower.pathBuilder()
                .addPath(new BezierLine(openAndCollect, openGate))
                .setLinearHeadingInterpolation(openAndCollect.getHeading(), openGate.getHeading())
                .build();



    }


    // Atualização Posições


    public void statePathUpdate(){
        switch (pathState) {
            case START_POS_TO_SHOOT:
                follower.followPath(driveStartToShoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5){
                    follower.followPath(driveShootToFirstCollect, true);
                    setPathState(PathState.SHOOT_TO_FIRST_COLLECT);
                }
                break;
            case SHOOT_TO_FIRST_COLLECT:
                if (!follower.isBusy()) {
                    follower.followPath(driveFirstCollectToOpenGate, true);
                    setPathState(PathState.FIRST_COLLECT_TO_OPEN_GATE);
                }
                break;

            case FIRST_COLLECT_TO_OPEN_GATE:
                if(!follower.isBusy()) {
                    follower.followPath(driveOpenGateToShoot, true);
                    setPathState(PathState.OPEN_GATE_TO_SHOOT);
                }
                break;

            case OPEN_GATE_TO_SHOOT:
                if(!follower.isBusy()) {
                    follower.followPath(driveShootToSecondCollect, true);
                    setPathState(PathState.SHOOT_TO_SECOND_COLLECT);
                }
                break;

            case SHOOT_TO_SECOND_COLLECT:
                if(!follower.isBusy()) {
                    follower.followPath(driveSecondCollectToShoot, true);
                    setPathState(PathState.SECOND_COLLECT_TO_SHOOT);
                }
                break;

            case SECOND_COLLECT_TO_SHOOT:
                if(!follower.isBusy()) {
                    follower.followPath(driveShootToOpenAndCollect, true);
                    setPathState(PathState.SHOOT_TO_OPEN_AND_COLLECT);
                }
                break;

            case OPEN_AND_COLLECT_TO_SHOOT:
                if(!follower.isBusy()) {
                    follower.followPath(driveShootToOpenAndCollect, true);
                    setPathState(PathState.SHOOT_TO_OPEN_AND_COLLECT_AND_OPEN_GATE);
                }
                break;

            case SHOOT_TO_OPEN_AND_COLLECT_AND_OPEN_GATE:
                if(!follower.isBusy()) {
                    follower.followPath(driveOpenAndCollectToOpenGate, true);
                    setPathState(PathState.OPEN_AND_COLLECT_TO_OPEN_GATE);
                }
                break;

            case OPEN_AND_COLLECT_TO_OPEN_GATE:
                if(!follower.isBusy()) {
                    follower.followPath(driveOpenGateToShoot, true);
                    telemetry.addLine("Rotina Finalizada");
                    setPathState(PathState.OPEN_GATE_TO_SHOOT);
                }
                break;


        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.START_POS_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(start);
    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
