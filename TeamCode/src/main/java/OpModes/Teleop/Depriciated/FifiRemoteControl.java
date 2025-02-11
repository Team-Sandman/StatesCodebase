//package OpModes.Teleop.Depriciated;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.IntakeOuttake;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//@TeleOp
//public class FifiRemoteControl extends OpMode {
//
//    // Used for teleop driving enhancement
//    private Follower follower;
//
//    IntakeOuttake jamocha = new IntakeOuttake();
//
//
//    //Drive Motors
//    private DcMotorEx leftFront;
//    private DcMotorEx leftRear;
//    private DcMotorEx rightFront;
//    private DcMotorEx rightRear;
//
//    enum State{
//        START,
//        //Start state to fit within 18 inches
//        PRE_SCORE_HIGH_CHAMBER,
//        //Line up to score at high chamber
//        SCORE_CHAMBER,
//        //Score at the high chamber
//        PRE_SCORE_LOW_CHAMBER,
//        //Line up to score at low chamber
//        PRE_SCORE_HIGH_BASKET,
//        //Line up to score at the low chamber
//        SCORE_HIGH_BASKET,
//        //Score at the high basket
//        PRE_SCORE_LOW_BASKET,
//        //Line up to score at the low basket
//        SCORE_LOW_BASKET,
//        //Score at the low basket
//        DRIVE,
//        //Pick up specimen from wall
//        PICK_UP_SPECIMEN,
//        GRAB_SPECIMEN,
//        //Retract slides for driving
//        SEARCH_FOR_SAMPLE,
//        //Deploys intake to search for a sample
//        PICK_UP_SAMPLE,
//        //Pick up sample from the submersible
//        TRANSFER_SAMPLE,
//        //Hand off sample from intake to outtake
//
//        EXPEL_SAMPLE,
//
//        EXPEL_WRONG_SAMPLE,
//        MANUAL_CONTROL,
//        //Driver takes manual control of slides
//        RECALIBRATE
//        //Rezero motor encoders
//    }
//    State state = State.START;
//@Override
//    public void init(){
//        //teleop driving enhancement initialization
//        follower = new Follower(hardwareMap);
//
//        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
//        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
//        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
//        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        follower.startTeleopDrive();
//
//
//        jamocha.initIntake(hardwareMap);
//        jamocha.initOuttake(hardwareMap);
//
//    }
//    @Override
//    public void loop(){
//        //Teleop Driving Enhanced
//        follower.setTeleOpMovementVectors(-gamepad1.right_stick_y + gamepad1.right_trigger - gamepad1.left_trigger, -gamepad1.right_stick_x, -gamepad1.left_stick_x);
//        follower.update();
//        telemetry.addData("State",state);
//        telemetry.update();
//        //State Change
//        if (gamepad1.a){
//            state = State.DRIVE;
//        }
//
//        //Prescore States
//        if (gamepad2.dpad_up){
//            state = State.PRE_SCORE_HIGH_CHAMBER;
//        }
//
//        if (gamepad2.dpad_down){
//            state = State.PRE_SCORE_LOW_CHAMBER;
//        }
//
//        if (gamepad2.dpad_right){
//            state = State.PRE_SCORE_HIGH_BASKET;
//        }
//
//        if (gamepad2.dpad_left){
//            state = State.PRE_SCORE_LOW_BASKET;
//        }
//
//        // pick up from wall
//
//
//        //Return to start state
//        if (gamepad2.start){
//            state = State.START;
//        }
//
//        if (gamepad1.back){
//            state = State.MANUAL_CONTROL;
//        }
//
//        switch (state){
//
//            case START:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armStowed();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftStowed();
//
//                if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_stick_y !=0 || gamepad1.right_trigger !=0 || gamepad1.left_trigger !=0){
//                    state = State.DRIVE;
//                }
//            break;
//
//
//
//
//            case PRE_SCORE_HIGH_CHAMBER:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftHighChamber();
//
//                //State Change
//                if (gamepad2.right_bumper){
//                    state = State.SCORE_CHAMBER;
//                }
//            break;
//
//            case SCORE_CHAMBER:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPickUp();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                //jamocha.liftStowed(); took this out to see if we can protect the servo.
//                //Now the driver must lower the lift with the right stick and then release the right bumper.
//                jamocha.liftManualControl(gamepad2.right_stick_y);
//
//                //State Change
//                if (!gamepad2.right_bumper){
//                state = State.DRIVE;
//                }
//
//                break;
//
//
//            case PRE_SCORE_LOW_CHAMBER:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftLowChamber();
//
//                //State Change
//                if (gamepad2.right_bumper){
//                    state = State.SCORE_CHAMBER;
//                }
//            break;
//
//
//            case PRE_SCORE_HIGH_BASKET:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftHighBasket();
//
//                //Sate Change
//                if (gamepad2.right_bumper){
//                state = State.SCORE_HIGH_BASKET;
//                }
//            break;
//
//            case SCORE_HIGH_BASKET:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftHighBasket();
//
//                //Sate Change
//                if (!gamepad2.right_bumper){
//                    state = State.DRIVE;
//                }
//            break;
//
//            case PRE_SCORE_LOW_BASKET:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftLowBasket();
//
//                //State Change
//                if (gamepad2.right_bumper){
//                    state = State.SCORE_LOW_BASKET;
//                }
//            break;
//
//            case SCORE_LOW_BASKET:
//                //four bar
//                jamocha.fourBarStowed();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftLowBasket();
//
//                //Sate Change
//                if (!gamepad2.right_bumper){
//                    state = State.DRIVE;
//                }
//                break;
//
//            case DRIVE:
//                //four bar
//                jamocha.fourBarTransfer();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPickUp();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftStowed();
//
//                //State Change
//                if (gamepad2.a){
//                    state=State.PICK_UP_SPECIMEN;
//                }
//
//                if (gamepad2.left_bumper){
//                    state = State.SEARCH_FOR_SAMPLE;
//                }
//
//                break;
//
//            case PICK_UP_SPECIMEN:
//                //four bar
//                jamocha.fourBarTransfer();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPickUp();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftStowed();
//
//                //State Change
//                if (gamepad2.b){
//                    state=State.DRIVE;
//                }
//
//                if (!gamepad2.a){
//                    state = State.GRAB_SPECIMEN;
//                }
//
//            break;
//
//            case GRAB_SPECIMEN:
//                //four bar
//                jamocha.fourBarTransfer();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armPreScore();
//
//                //claw
//                jamocha.clawClosed();
//
//                //lift
//                jamocha.liftStowed();
//
//                if (gamepad2.b){
//                    state=State.DRIVE;
//                }
//            break;
//
//
//
//            case SEARCH_FOR_SAMPLE:
//                //four bar
//                jamocha.fourBarSearch();
//                jamocha.fourBarPitchSearch();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);
//
//                //arm
//                jamocha.armTransfer();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftTransfer();
//
//                //State Change
//                if (gamepad2.a){
//                    state = State.PICK_UP_SAMPLE;
//                }
//
//                if (gamepad2.y){
//                    state = State.EXPEL_WRONG_SAMPLE;
//                }
//
//                if (gamepad2.b){
//                    state = State.DRIVE;
//                }
//            break;
//
//            case PICK_UP_SAMPLE:
//                //four bar
//                jamocha.fourBarDown();
//                jamocha.fourBarPitchSearch();
//
//                //intake
//                jamocha.intakeIn();
//
//                //horizontal slides
//                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);
//
//                //arm
//                jamocha.armTransfer();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftTransfer();
//
//                //State Change
//                if (!gamepad2.a){
//                    state = State.TRANSFER_SAMPLE;
//                }
//            break;
//
//            case TRANSFER_SAMPLE:
//               if (jamocha.horizontalSlidesReturnPosition() * jamocha.horizontalSlidesReturnPosition()<5000) {
//                   //four bar
//                   jamocha.fourBarTransfer();
//                   jamocha.fourBarPitchTransfer();
//
//                   //intake
//                   jamocha.intakeOff();
//               }
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armTransfer();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftTransfer();
//
//                // State Change
//                if (gamepad2.a){
//                    state = State.EXPEL_SAMPLE;
//                }
//                if (gamepad2.left_bumper){
//                    state = State.SEARCH_FOR_SAMPLE;
//                }
//            break;
//
//            case EXPEL_SAMPLE:
//                //four bar
//                jamocha.fourBarTransfer();
//                jamocha.fourBarPitchTransfer();
//
//                //intake
//                jamocha.intakeOut();
//
//                //horizontal slides
//                jamocha.horizontalSlidesHome();
//
//                //arm
//                jamocha.armTransfer();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftTransfer();
//
//                //State Change
//                if (!gamepad2.a){
//                    state = State.GRAB_SPECIMEN;
//                }
//                break;
//
//            case EXPEL_WRONG_SAMPLE:
//                //four bar
//                jamocha.fourBarSearch();
//                jamocha.fourBarPitchSearch();
//
//                //intake
//                jamocha.intakeOut();
//
//                //horizontal slides
//                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);
//
//                //arm
//                jamocha.armTransfer();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftTransfer();
//
//                //State change
//                if (!gamepad2.y){
//                    state = State.SEARCH_FOR_SAMPLE;
//                }
//                break;
//
//            case MANUAL_CONTROL:
//                jamocha.fourBarTransfer();
//                jamocha.fourBarPitchStowed();
//
//                //intake
//                jamocha.intakeOff();
//
//                //horizontal slides
//                jamocha.horizontalSlidesManualControl(gamepad2.left_stick_y);
//
//                //arm
//                jamocha.armPickUp();
//
//                //claw
//                jamocha.clawOpen();
//
//                //lift
//                jamocha.liftManualControl(gamepad2.right_stick_y);
//
//                //State Change
//                if (gamepad2.back){
//                    state = State.RECALIBRATE;
//                }
//                break;
//
//            case RECALIBRATE:
//                //horizontal slides
//                jamocha.recalibrateHorizontalSlide();
//
//                //lift
//                jamocha.recalibrateLift();
//
//                //State Change
//                state = State.START;
//            break;
//
//        }
//
//
///*
//        if (gamepad2.y){
//            jamocha.fourBarTransfer();
//        }
//
//        if (gamepad2.a){
//            jamocha.fourBarStowed();
//        }
//
//        if (gamepad2.b){
//            jamocha.fourBarSearch();
//        }
//
//        if (gamepad2.x){
//            jamocha.fourBarDown();
//        }
//
//        if (gamepad2.dpad_up){
//            jamocha.fourBarPitchSearch();
//        }
//
//        if (gamepad2.dpad_down){
//            jamocha.fourBarPitchStowed();
//        }
//
//        if (gamepad2.dpad_right){
//            jamocha.armTransfer();
//        }
//        if (gamepad2.dpad_left){
//            jamocha.armPreScore();
//        }
//        if (gamepad2.right_trigger>.25){
//            jamocha.armPickUp();
//        }
//
//        if (gamepad2.right_bumper){
//            jamocha.intakeIn();
//        }
//        else if (gamepad2.left_bumper){
//            jamocha.intakeOut();
//        }
//        else {
//            jamocha.intakeOff();
//        }
//        if (gamepad2.left_trigger<.5){
//            jamocha.clawOpen();
//        }
//        else{
//            jamocha.clawClosed();
//        }
//
//        if (gamepad1.dpad_up){
//            jamocha.liftHighChamber();
//        }
//     */
//    }
//
//
//
//}
