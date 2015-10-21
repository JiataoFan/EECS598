//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:
*/
function quaternion_from_axisangle(currentJoint) {
	
	var quaternion = [];
	
	quaternion[0] = Math.cos(robot.joints[currentJoint].angle / 2);
	
	quaternion[1] = robot.joints[currentJoint].axis[0] * 
					Math.sin(robot.joints[currentJoint].angle / 2);
	
	quaternion[2] = robot.joints[currentJoint].axis[1] * 
					Math.sin(robot.joints[currentJoint].angle / 2);
					
	quaternion[3] = robot.joints[currentJoint].axis[2] * 
					Math.sin(robot.joints[currentJoint].angle / 2);
	
	return quaternion;

}

function quaternion_normalize(quaternion) {
	
	var normalizedQuaternion = [];
	
	var magnitude = 0;
	
	for(var i = 0; i < quaternion.length; i++) {
		
		magnitude += quaternion[i] * quaternion[i];
		
	}
	
	magnitude = Math.sqrt(magnitude);
	
	for(var i = 0; i < quaternion.length; i++) {

		normalizedQuaternion[i] = quaternion[i] / magnitude;
		
	}
	
	return normalizedQuaternion;

}

function quaternion_multiply(quaternionA, quaternionB) {
	
	var productQuaternion = [];
	
	productQuaternion[0] = quaternionA[0] * quaternionB[0]
							- quaternionA[1] * quaternionB[1] 
							- quaternionA[2] * quaternionB[2] 
							- quaternionA[3] * quaternionB[3];

	productQuaternion[1] = quaternionA[0] * quaternionB[1]
							+ quaternionA[1] * quaternionB[0] 
							+ quaternionA[2] * quaternionB[3] 
							- quaternionA[3] * quaternionB[2];

	productQuaternion[2] = quaternionA[0] * quaternionB[2]
							- quaternionA[1] * quaternionB[3] 
							+ quaternionA[2] * quaternionB[0] 
							+ quaternionA[3] * quaternionB[1];

	productQuaternion[3] = quaternionA[0] * quaternionB[3]
							+ quaternionA[1] * quaternionB[2]
							- quaternionA[2] * quaternionB[1] 
							+ quaternionA[3] * quaternionB[0];
	
	return productQuaternion;

}

function quaternion_to_rotation_matrix(quaternion) {
	
	var rotationMatrix = generate_identity();
	
	rotationMatrix[0][0] = 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
	
	rotationMatrix[0][1] = 2 * (quaternion[1] * quaternion[2] - quaternion[0] * quaternion[3]);
	
	rotationMatrix[0][2] = 2 * (quaternion[0] * quaternion[2] + quaternion[1] * quaternion[3]);
	
	rotationMatrix[1][0] = 2 * (quaternion[1] * quaternion[2] + quaternion[0] * quaternion[3]);
	
	rotationMatrix[1][1] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[3] * quaternion[3]);
	
	rotationMatrix[1][2] = 2 * (quaternion[2] * quaternion[3] - quaternion[0] * quaternion[1]);
	
	rotationMatrix[2][0] = 2 * (quaternion[1] * quaternion[3] - quaternion[0] * quaternion[2]);
	
	rotationMatrix[2][1] = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
	
	rotationMatrix[2][2] = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
	
	return rotationMatrix;
	
}