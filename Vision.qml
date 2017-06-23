import ThymioAR 1.0

VisionVideoFilter {
	id: self
	// make these properties notifiable
	property alias robot: self.robot
	property alias landmarks: self.landmarks
	property matrix4x4 robotPose: landmarks.length > 0 ? landmarks[0].pose.inverted().times(robot.pose) : robot.pose

    property bool leastOneMarkerActive : {
        var ret = false
        for(var i = 0; i < landmarks.length; i++)
            ret  = landmarks[i].active || ret
        return ret
        }
}
