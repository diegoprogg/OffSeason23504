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
public class Autonomous extends OpMode {
    private Follower follower;

    private Timer pathTimer, opModeTimer;

    private final Pose controlPointOpenGateToSecondLine = new Pose(66.56584530023554, 74.00527665421346);
    private final Pose secondControlPointOpenGateToSecondLine = new Pose(86.52067396067167, 53.115868554524994);

    // Estados
    public enum PathState{
        // START POS -> FINAL POS
        START_POS_TO_SHOOT,
        SHOOT_PRELOAD,
        SHOOT_TO_COLLECTION_POINT,
        COLLECTION_POINT_TO_SHOOT,
        SHOOT_TO_OPEN_GATE,

        OPEN_GATE_TO_SECOND_LINE,

        SECOND_LINE_TO_SHOOT,

        OPEN_GATE_TO_POSITION_COLLECTION,


        POSITION_COLLECTION_TO_COLLECTION_SECOND_LINE,
        COLLECTION_SECOND_LINE_TO_SHOOT,
        SHOOT_TO_POSITION_COLLECTION_THIRD_LINE,

        POSITION_COLLECTION_THIRD_LINE_TO_COLLECTION_THIRD_LINE,

        COLLECTION_THIRD_LINE_TO_SHOOT;


    }



    PathState pathState;

    // Posições

    private final Pose startPose = new Pose(111.54205607476635, 133.190031152648, Math.toRadians(90));

    private final Pose shootPose = new Pose(82.44859813084112, 81.62383177570092, Math.toRadians(45));

    private final Pose collectionPose = new Pose(124.74922118380061, 81.54984423676012, Math.toRadians(0));

    private final Pose openGatePose = new Pose(126.95260577024516, 73.20922529281542, Math.toRadians(90));

    private final Pose positionCollection = new Pose(70.97040498442367, 58.848130841121495, Math.toRadians(180));

    private final Pose collectionSecondLinePose = new Pose(126.25073592056862, 58.32001960439656, Math.toRadians(0));

    private final Pose positionCollectionThirdLinePose = new Pose(58.20383156578464, 35.743653787141625, Math.toRadians(180));

    private final Pose collectionThirdLinePose = new Pose(16.84982280553337, 34.85614598169521, Math.toRadians(180));



    private PathChain
            driveStartPoseToShootPose,
            driveShootPoseToCollectionPose,
            driveCollectionPoseToShootPose,

    driveShootPoseToOpenGatePose,

    driveOpenGateToPositionCollection,

    drivePositionCollectionToSecondLinePose,

    driveSecondLineToShoot,

    driveShootPoseToPositionCollectionThirdLine,

    drivePositionCollectionThirdLineToCollectionThirdLine,

    driveCollectionThirdLineToShoot,

    driveOpenGateToSecondLine;




    public void buildPaths(){
        driveStartPoseToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToCollectionPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, collectionPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), collectionPose.getHeading())
                .build();

        driveCollectionPoseToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(collectionPose, shootPose))
                .setLinearHeadingInterpolation(collectionPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToOpenGatePose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, openGatePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), openGatePose.getHeading())
                .build();

        driveOpenGateToSecondLine = follower.pathBuilder()
                .addPath(new BezierCurve(openGatePose, controlPointOpenGateToSecondLine, secondControlPointOpenGateToSecondLine, collectionSecondLinePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), collectionSecondLinePose.getHeading())
                .build();



        driveOpenGateToPositionCollection = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, positionCollection))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), positionCollection.getHeading())
                .build();

        drivePositionCollectionToSecondLinePose = follower.pathBuilder()
                .addPath(new BezierLine(positionCollection, collectionSecondLinePose))
                .setLinearHeadingInterpolation(positionCollection.getHeading(), collectionSecondLinePose.getHeading())
                .build();

        driveSecondLineToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collectionSecondLinePose, shootPose))
                .setLinearHeadingInterpolation(collectionSecondLinePose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPoseToPositionCollectionThirdLine = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, positionCollectionThirdLinePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), positionCollectionThirdLinePose.getHeading())
                .build();

        drivePositionCollectionThirdLineToCollectionThirdLine = follower.pathBuilder()
                .addPath(new BezierLine(positionCollectionThirdLinePose, collectionThirdLinePose))
                .setLinearHeadingInterpolation(positionCollectionThirdLinePose.getHeading(), collectionThirdLinePose.getHeading())
                .build();

        driveCollectionThirdLineToShoot = follower.pathBuilder()
                .addPath(new BezierLine(collectionThirdLinePose, shootPose))
                .setLinearHeadingInterpolation(collectionThirdLinePose.getHeading(), shootPose.getHeading())
                .build();
    }


    // Atualização Posições


    public void statePathUpdate(){
        switch (pathState) {
            case START_POS_TO_SHOOT:
                follower.followPath(driveStartPoseToShootPose, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2){
                    follower.followPath(driveShootPoseToCollectionPose, true);
                    telemetry.addLine("Primeiro Caminho Feito");
                    setPathState(PathState.SHOOT_TO_COLLECTION_POINT);
                }
                break;
            case SHOOT_TO_COLLECTION_POINT:
                if (!follower.isBusy()) {
                    follower.followPath(driveCollectionPoseToShootPose, true);
                    setPathState(PathState.COLLECTION_POINT_TO_SHOOT);
                }
                break;

            case COLLECTION_POINT_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(driveShootPoseToOpenGatePose, true);
                    setPathState(PathState.SHOOT_TO_OPEN_GATE);
                }
                break;

            case SHOOT_TO_OPEN_GATE:
                if (!follower.isBusy()){
                    follower.followPath(driveOpenGateToSecondLine, true);
                    setPathState(PathState.OPEN_GATE_TO_SECOND_LINE);
                }
                break;
            case OPEN_GATE_TO_SECOND_LINE:
                if(!follower.isBusy()){
                    follower.followPath(driveSecondLineToShoot, true);
                    setPathState(PathState.SECOND_LINE_TO_SHOOT);
                }
                break;



//            case OPEN_GATE_TO_POSITION_COLLECTION:
//                if (!follower.isBusy()){
//                    follower.followPath(drivePositionCollectionToSecondLinePose, true);
//                    setPathState(PathState.POSITION_COLLECTION_TO_COLLECTION_SECOND_LINE);
//                }
//                break;
//            case POSITION_COLLECTION_TO_COLLECTION_SECOND_LINE:
//                if (!follower.isBusy()){
//                    follower.followPath(driveSecondLineToShoot, true);
//                    setPathState(PathState.COLLECTION_SECOND_LINE_TO_SHOOT);
//                }
//                break;
//
//            case COLLECTION_SECOND_LINE_TO_SHOOT:
//                if (!follower.isBusy()){
//                    follower.followPath(driveShootPoseToPositionCollectionThirdLine);
//                    setPathState(PathState.SHOOT_TO_POSITION_COLLECTION_THIRD_LINE);
//                }
//                break;
//            case SHOOT_TO_POSITION_COLLECTION_THIRD_LINE:
//                if (!follower.isBusy()){
//                    follower.followPath(drivePositionCollectionThirdLineToCollectionThirdLine);
//                    setPathState(PathState.POSITION_COLLECTION_THIRD_LINE_TO_COLLECTION_THIRD_LINE);
//                }
//                break;
//
//            case POSITION_COLLECTION_THIRD_LINE_TO_COLLECTION_THIRD_LINE:
//                if (!follower.isBusy()){
//                    follower.followPath(driveCollectionThirdLineToShoot);
//                    setPathState(PathState.COLLECTION_THIRD_LINE_TO_SHOOT);
//                }
//                break;
//            default:
//                telemetry.addLine("Sem comandos para caminho");
//                break;
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
        follower.setPose(startPose);
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
