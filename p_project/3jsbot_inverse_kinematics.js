//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {

	// compute joint angle controls to move location on specified link to Cartesian location
	if (update_ik) {

		iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
		endeffector_geom.visible = true;
		target_geom.visible = true;

	} else {

		endeffector_geom.visible = false;
		target_geom.visible = false;

	}

	update_ik = false;

	var endEffectorGlobalPosition = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_local_pos);
	var endEffectorGlobalPosition4By4 = generate_identity();
	endEffectorGlobalPosition4By4[0][3] = endEffectorGlobalPosition[0][0];
	endEffectorGlobalPosition4By4[1][3] = endEffectorGlobalPosition[1][0];
	endEffectorGlobalPosition4By4[2][3] = endEffectorGlobalPosition[2][0];
	var endeffector_mat = matrix_2Darray_to_threejs(endEffectorGlobalPosition4By4);
	simpleApplyMatrix(endeffector_geom,endeffector_mat);

	var targetGlobalPosition4By4 = generate_identity();
	targetGlobalPosition4By4[0][3] = target_pos[0][0];
	targetGlobalPosition4By4[1][3] = target_pos[1][0];
	targetGlobalPosition4By4[2][3] = target_pos[2][0];
	var target_mat = matrix_2Darray_to_threejs(targetGlobalPosition4By4);
	simpleApplyMatrix(target_geom,target_mat);

}

function iterate_inverse_kinematics(targetPosition, endEffectorJoint, endEffectorLocalPosition) {

	var jacobianMatrix = [];
	for (var i = 0; i < 6; i++) {

		jacobianMatrix[i] = [];

	}

	var endEffectorGlobalPosition = matrix_multiply(robot.joints[endEffectorJoint].xform, endEffectorLocalPosition);

	var endEffectorGlobalPositionT = [];
	for (var i = 0; i < 3; i++) {

		endEffectorGlobalPositionT[i] = endEffectorGlobalPosition[i][0];

	}

	generateJacobianMatrix(endEffectorJoint, jacobianMatrix, endEffectorGlobalPositionT);

	var endEffectorTargetPositionDifference = [];
	for (var i = 0; i < 3; i++) {
	
		endEffectorTargetPositionDifference[i] = [targetPosition[i][0] - endEffectorGlobalPosition[i][0]];
		
	}
	for (var i = 3; i < 6; i++) {

		endEffectorTargetPositionDifference[i] = [0];

	};

	var angularDisplacement = matrix_multiply(matrix_transpose(jacobianMatrix), endEffectorTargetPositionDifference);

	renderAngularDisplacement(endEffectorJoint, angularDisplacement);

}

function generateJacobianMatrix(currentJoint, jacobianMatrix, endEffectorGlobalPositionT) {

	if (typeof robot.joints[currentJoint] === 'undefined') {

		if (robot.links[currentJoint].name !== robot.base ) {

			generateJacobianMatrix(robot.links[currentJoint].parent, jacobianMatrix, endEffectorGlobalPositionT);

		}

	} else {

		var jointLocalPosition = [];
		for (var i = 0; i < 3; i++) {

			jointLocalPosition[i] = [0];

		}
		jointLocalPosition[3] = [1];

		var jointGlobalPosition = matrix_multiply(robot.joints[currentJoint].xform, jointLocalPosition);

		var jointGlobalPositionT = [];
		for (var i = 0; i < 3; i++) {

			jointGlobalPositionT[i] = jointGlobalPosition[i][0];

		}

		var jointLocalAxis = [];
		for (var i = 0; i < 3; i++) {

			jointLocalAxis[i] = [robot.joints[currentJoint].axis[i]];

		}
		jointLocalAxis[3] = [1];

		var jointGlobalAxis = matrix_multiply(robot.joints[currentJoint].xform, jointLocalAxis);

		var jointGlobalAxisT = [];
		for (var i = 0; i < 3; i++) {

			jointGlobalAxisT[i] = jointGlobalAxis[i][0];

		}

		var jointEndEffectorPositionDifferenceT = [];
		for (var i = 0; i < 3; i++) {

			jointEndEffectorPositionDifferenceT[i] = endEffectorGlobalPositionT[i] - jointGlobalPositionT[i];

		}

		var jacobianUpperRowsT = vector_cross(jointGlobalAxisT, jointEndEffectorPositionDifferenceT);

		for (var i = 0; i < 3; i++) {

			jacobianMatrix[i].unshift(jacobianUpperRowsT[i]);

		}

		for (var i = 0; i < 3; i++) {

			jacobianMatrix[i + 3].unshift(jointGlobalAxisT[i]);

		}

		generateJacobianMatrix(robot.joints[currentJoint].parent, jacobianMatrix, endEffectorGlobalPositionT);

	}

}

function renderAngularDisplacement(currentJoint, angularDisplacement) {

	if (typeof robot.joints[currentJoint] === 'undefined') {

		if (robot.links[currentJoint].name !== robot.base ) {

			renderAngularDisplacement(robot.links[currentJoint].parent, angularDisplacement);

		}

	} else { 

		robot.joints[currentJoint].control = 0.1 * (angularDisplacement.pop())[0];

		renderAngularDisplacement(robot.joints[currentJoint].parent, angularDisplacement);

	}

}