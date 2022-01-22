package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2d;

public enum ReferenceFrame {
    NONE(new Pose2d()),
    START(new Pose2d()),
    SCALE(new Pose2d());

    private Pose2d mReferenceFrame;

    ReferenceFrame(Pose2d mReferenceFrame) {
        this.mReferenceFrame = mReferenceFrame;
    }

    public Pose2d getReferenceFrame() {
        return this.mReferenceFrame;
    }
}
