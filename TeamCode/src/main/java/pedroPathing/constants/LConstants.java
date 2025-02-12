package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00297;
        ThreeWheelConstants.strafeTicksToInches = 0.0029;
        ThreeWheelConstants.turnTicksToInches = 0.00288;
        ThreeWheelConstants.leftY = 7;
        ThreeWheelConstants.rightY = -7;
        ThreeWheelConstants.strafeX = 4.875;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
    /* This is code for the pinpoint when we switch
    static {
    PinpointConstants.forwardY = 1;
    PinpointConstants.strafeX = -2.5;
    PinpointConstants.distanceUnit = DistanceUnit.INCH;
    PinpointConstants.hardwareMapName = "pinpoint";
    PinpointConstants.useYawScalar = false;
    PinpointConstants.yawScalar = 1.0;
    PinpointConstants.useCustomEncoderResolution = false;
    PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    PinpointConstants.customEncoderResolution = 13.26291192;
    PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
     */
}




