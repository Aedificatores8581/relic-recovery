package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

/**
 * Conjured into existence by The Saminator on 03-06-2018.
 */

@Disabled
@Autonomous(name = "DriveBot Angles", group = "this is a test")
public class DriveBotTestAngles extends DriveBotTestTemplate {

    Acceleration gravity;
    @Override
    public void loop() {
        gravity = imu.getGravity();
        Spherical3D angles = cartesianToSpherical(new Cartesian3D(gravity.xAccel, gravity.yAccel, gravity.zAccel));

        telemetry.addData("(NAV) Status", imu.getSystemStatus().toShortString());
        telemetry.addData("(NAV) Calib.", imu.getCalibrationStatus());

        telemetry.addData("Theta angle ( acos (z / sqrt(x^2 + y^2 + z^2)) )", angles.theta);
        telemetry.addData("Phi angle ( atan (y / x) )", angles.phi);
        telemetry.addData("X gravity", gravity.xAccel);
        telemetry.addData("Y gravity", gravity.yAccel);
        telemetry.addData("Z gravity", gravity.zAccel);
    }

    public Spherical3D cartesianToSpherical(Cartesian3D cartesian) {
        double
                x2 = cartesian.x * cartesian.x,
                y2 = cartesian.y * cartesian.y,
                z2 = cartesian.z * cartesian.z;
        double r = Math.sqrt(x2 + y2 + z2);
        double theta = Math.acos(cartesian.z / r) * org.firstinspires.ftc.teamcode.Constants.RADS_TO_DEGS;
        double phi = Math.atan2(cartesian.y, cartesian.x) * org.firstinspires.ftc.teamcode.Constants.RADS_TO_DEGS;
        return new Spherical3D(r, theta, phi);
    }

}
