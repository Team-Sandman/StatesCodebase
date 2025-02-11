//
//package OpModes.Autos;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import Common.Robot.IntakeOuttakeV2;
//@Config
//public class  ififBucketAuto extends OpMode {
//    private Timer pathTimer;
//    private int pathState;
//    private Follower follower;
//    private Path  eat1;
//    private PathChain preload, score2, eat2, score3, backOff;
//    private final Pose startPose = new Pose(8.3,111.3,Math.toRadians(90));//x was 6.7291
//    private final Pose scorePose = new Pose(14.4,129.6-3,Math.toRadians(135));
//    private final Pose pickup1 = new Pose(35.4-5,122-.5,Math.toRadians(178-2));//was x=66.8411, y=25.7944; then 65.7196,28.4861, then (61.9065,31.4019)
//    private final Pose pickup1Control = new Pose(21.98,123.6,Math.toRadians(180));//was 2.6916,53.1589
//    private final Pose pickup2 = new Pose(34.99-4.5, 132.1-3.5, Math.toRadians(178-4));//was x=68.6355, y=23.3271, then (77.3832, 24.8972), then 69.0841, 20.8598, then (61.0093, 18.3925), then (62.1308, 14.3551), then (60.1121, 17.4953)
//    private final Pose End = new Pose(20, 125, Math.toRadians(135));
//
//    //private final Pose sample3 = new Pose(57.4206-1, 10.9907+1, Math.toRadians(180)); //was (x,y)(71.3271, 9.1963), then (71.1028, 10.9907), then (62.8037, 9.4206), then (59.8879, 9.8692), then (59.43925, 10.5421)
//    //private final Pose sample3Control = new Pose(57.1963,25.7944,Math.toRadians(180));//was (80.0748,24.4486)
//
//
//    IntakeOuttake jamocha = new IntakeOuttake();
//
//    //Lift lift = new Lift();
//
//    public void pathBuilder() {
//        preload = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                .build();
//
//        eat1 = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(pickup1Control), new Point(pickup1)));
//        eat1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1.getHeading());
//
//        score2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup1), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup1.getHeading(), scorePose.getHeading())
//                .build();
//
//        eat2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2.getHeading())
//                .build();
//
//        score3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2), new Point(scorePose.getX()+1, scorePose.getY()-.75)))
//                .setLinearHeadingInterpolation(pickup2.getHeading(), scorePose.getHeading())
//                .build();
//
//        backOff = follower.pathBuilder()
//                .addPath(new BezierLine(new Point (scorePose), new Point(End)))
//                .setConstantHeadingInterpolation(scorePose.getHeading())
//                .build();
//    }
//    public void autoPathUpdate(){
//        switch(pathState){
//            case 0:
//                jamocha.fourBarPitchTransfer();
//                jamocha.fourBarTransfer();
//                jamocha.horizontalSlidesHome();
//                jamocha.armPickUp();
//                jamocha.clawClosed();
//                jamocha.liftAutoHighBasket(); //was -1410
//                pathTimer.resetTimer();
//                setPathState(1000);
//                break;
//            case 1000:
//                if(pathTimer.getElapsedTimeSeconds()>3.25) {
//                    follower.setMaxPower(0.85);
//                    follower.followPath(preload, true);
//                    setPathState(100);
//                }
//                break;
//            case 100:
//                if(follower.atParametricEnd()){
//                    jamocha.clawOpen();
//                   //jamocha.
//                    // lift.autoLift(0);
//                    setPathState(101);
//                }
//                break;
//            case 101:
//                if (pathTimer.getElapsedTimeSeconds()>1){
//                    follower.followPath(eat1);
//                    setPathState(102);
//                }
//                break;
//            case 102:
//                if(pathTimer.getElapsedTimeSeconds()>.9){
//                    jamocha.liftTransfer();
//                    setPathState(103);
//                }
//                break;
//            case 103:
//                if (pathTimer.getElapsedTimeSeconds()>4-2) {
//                    jamocha.armTransfer();
//                    jamocha.fourBarSearch();
//                    jamocha.fourBarPitchSearch();
//                    setPathState(104);
//                }
//                break;
//            case 104:
//                if (pathTimer.getElapsedTimeSeconds()>2){
//                    jamocha.fourBarDown();
//                    jamocha.fourBarPitchSearch();
//                    jamocha.intakeIn();
//                    setPathState(1);
//                }
//                break;
//            /*case 1:
//                if(follower.getCurrentTValue()>0.995){
//                    follower.setMaxPower(0.6);
//                    follower.followPath(eat1);
//                    setPathState(11);
//                }
//                break;*/
//            case 1:
//                if(pathTimer.getElapsedTimeSeconds()>2-1){
//                    //jamocha.fourBarDown();
//                    jamocha.intakeOff();
//                    jamocha.fourBarTransfer();
//                    jamocha.fourBarPitchTransfer();
//                    setPathState(121);
//                }
//                break;
//            case 121:
//                if (pathTimer.getElapsedTimeSeconds()>1.5-1) {
//                    jamocha.intakeOut();
//                    setPathState(122);
//                }
//                break;
//                    case 122:
//                        if (pathTimer.getElapsedTimeSeconds()>1-.5){
//                    jamocha.clawClosed();
//                    jamocha.armPickUp();
//                    setPathState(123);
//                }
//                break;
//            case 123:
//                if(pathTimer.getElapsedTimeSeconds()>.5){
//                    jamocha.intakeOff();
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if(pathTimer.getElapsedTimeSeconds()>.1){
//                    jamocha.liftAutoHighBasket();
//                    setPathState(2);
//                }
//                break;
//            case 2:
//                if(pathTimer.getElapsedTimeSeconds()>2.5){
//                    follower.setMaxPower(0.85);//was.7
//                    follower.followPath(score2);
//                    setPathState(21);
//                }
//                break;
//            case 21:
//                //if (follower.atParametricEnd()
//                if(follower.atParametricEnd()){
//                    jamocha.clawOpen();
//                    // lift.autoLift(0);
//                    setPathState(22);
//                }
//                break;
//            case 22:
//                if (pathTimer.getElapsedTimeSeconds()>1){
//                    follower.followPath(eat2);
//                    setPathState(221);
//                }
//                break;
//            case 221:
//                if(pathTimer.getElapsedTimeSeconds()>.9){
//                    jamocha.liftTransfer();
//                    setPathState(222);
//                }
//                break;
//            case 222:
//                if (pathTimer.getElapsedTimeSeconds()>4-2) {
//                    jamocha.armTransfer();
//                    jamocha.fourBarSearch();
//                    jamocha.fourBarPitchSearch();
//                    setPathState(223);
//                }
//                break;
//            case 223:
//                if (pathTimer.getElapsedTimeSeconds()>2){
//                    jamocha.fourBarDown();
//                    jamocha.fourBarPitchSearch();
//                    jamocha.intakeIn();
//
//
//                    setPathState(224);
//                }
//                break;
//            case 224:
//                if(pathTimer.getElapsedTimeSeconds()>.55+.45){
//                    //jamocha.fourBarDown();
//                    jamocha.intakeOff();
//                    jamocha.fourBarTransfer();
//                    jamocha.fourBarPitchTransfer();
//                    setPathState(225);
//                }
//                break;
//            case 225:
//                if (pathTimer.getElapsedTimeSeconds()>1.5-1) {
//                    jamocha.intakeOut();
//                    setPathState(226);
//                }
//                break;
//            case 226:
//                if (pathTimer.getElapsedTimeSeconds()>1){
//                    jamocha.clawClosed();
//                    jamocha.armPickUp();
//                    setPathState(227);
//                }
//                break;
//            case 227:
//                if(pathTimer.getElapsedTimeSeconds()>.5){
//                    jamocha.intakeOff();
//                    setPathState(229);
//                }
//                break;
//            case 229:
//                if(pathTimer.getElapsedTimeSeconds()>1-.5){
//                    jamocha.liftAutoHighBasket();
//                    setPathState(3);
//                }
//                break;
//            case 3:
//                if(pathTimer.getElapsedTimeSeconds()>2.5+.5){
//                    follower.setMaxPower(0.85);//was .7
//                    follower.followPath(score3);
//                    setPathState(31);
//                }
//                break;
//            case 31:
//                if(follower.atParametricEnd()){
//                    jamocha.clawOpen();
//                    // lift.autoLift(0);
//                    setPathState(32);
//                }
//                break;
//            case 32:
//                if (/*follower.getCurrentTValue()>.995*/pathTimer.getElapsedTimeSeconds()>.5+1.5){
//                    follower.setMaxPower(0.85);
//                    follower.followPath(backOff);
//                    setPathState(-1);
//                }
//                break;
//           /* case 33:
//                if(pathTimer.getElapsedTimeSeconds()>1.25){
//                    jamocha.liftStowed();
//                    jamocha.fourBarPitchSearch();
//                    jamocha.fourBarSearch();
//                    jamocha.fourBarDown();
//                    jamocha.armTransfer();
//                    setPathState(34);
//                }
//                break;
//            case 34:
//                if(pathTimer.getElapsedTimeSeconds()>2){
//                    //jamocha.fourBarDown();
//                    jamocha.fourBarTransfer();
//                    jamocha.fourBarPitchTransfer();
//                    jamocha.intakeOut();
//                    jamocha.clawClosed();
//                    jamocha.armPickUp();
//                    setPathState(4);
//                }
//                break;
//            case 4:
//                if(follower.getCurrentTValue()>0.995){
//                    follower.setMaxPower(0.85);
//                    follower.followPath(score3);
//                    setPathState(-1);
//                }
//                break;*/
//
//        }
//    }
//    public void setPathState(int pState) {
//        pathState = pState;
//        pathTimer.resetTimer();
//    }
//    @Override
//    public void init() {
//        pathTimer = new Timer();
//        // opmodeTimer = new Timer();
//
//        //opmodeTimer.resetTimer();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        pathBuilder();
//
//        jamocha.initIntake(hardwareMap);
//        jamocha.initOuttake(hardwareMap);
//
//        //lift.initLiftAuto(hardwareMap);
//
//       // claw.initSpecimen_Claw(hardwareMap);
//
//       // claw.Specimen_Claw_Closed();
//       // claw.Closer_Servo_Closed();
//
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autoPathUpdate();
//
//        // Feedback to Driver Hub
//        telemetry.addData("path state", pathState);
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.update();
//    }
//    @Override
//    public void start(){
//        setPathState(0);
//    }
//}
