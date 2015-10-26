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

	/*
	Draw the end effector
	*/
	var endEffectorGlobalPosition = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_local_pos);
	var endEffectorGlobalPosition4By4 = generate_identity();
	endEffectorGlobalPosition4By4[0][3] = endEffectorGlobalPosition[0][0];
	endEffectorGlobalPosition4By4[1][3] = endEffectorGlobalPosition[1][0];
	endEffectorGlobalPosition4By4[2][3] = endEffectorGlobalPosition[2][0];
	var endeffector_mat = matrix_2Darray_to_threejs(endEffectorGlobalPosition4By4);
	simpleApplyMatrix(endeffector_geom,endeffector_mat);

	/*
	Draw the target
	*/
	var targetGlobalPosition4By4 = generate_identity();
	targetGlobalPosition4By4[0][3] = target_pos[0][0];
	targetGlobalPosition4By4[1][3] = target_pos[1][0];
	targetGlobalPosition4By4[2][3] = target_pos[2][0];
	var target_mat = matrix_2Darray_to_threejs(targetGlobalPosition4By4);
	simpleApplyMatrix(target_geom,target_mat);

}

/*
Perform inverse kinematics
*/
function iterate_inverse_kinematics(targetPosition, endEffectorJoint, endEffectorLocalPosition) {

	/*
	Define the jacobian matrix
	*/
	var jacobianMatrix = [];
	for (var i = 0; i < 6; i++) {

		jacobianMatrix[i] = [];

	}

	/*
	Generate end effector's global position
	*/
	var endEffectorGlobalPosition = matrix_multiply(robot.joints[endEffectorJoint].xform, endEffectorLocalPosition);

	/*
	Transpose of the end effector's global position
	*/
	var endEffectorGlobalPositionT = [];
	for (var i = 0; i < 3; i++) {

		endEffectorGlobalPositionT[i] = endEffectorGlobalPosition[i][0];

	}

	/*
	Generate jacobian matrix
	*/
	generateJacobianMatrix(endEffectorJoint, jacobianMatrix, endEffectorGlobalPositionT);

	/*
	The position difference between the end effector and the target
	*/
	var endEffectorTargetPositionDifference = [];
	for (var i = 0; i < 3; i++) {
	
		endEffectorTargetPositionDifference[i] = [targetPosition[i][0] - endEffectorGlobalPosition[i][0]];
		
	}
	for (var i = 3; i < 6; i++) {

		endEffectorTargetPositionDifference[i] = [0];

	};

	/*
	Calculate the angular displacement by multiplying jacobian matrix by the end effector-target position difference
	*/
	var angularDisplacement = matrix_multiply(matrix_transpose(jacobianMatrix), endEffectorTargetPositionDifference);

	/*
	Assign the angular displacement to each joint
	*/
	renderAngularDisplacement(endEffectorJoint, angularDisplacement);

}

/*
Gnerate the jacobian matrix
*/
function generateJacobianMatrix(currentJoint, jacobianMatrix, endEffectorGlobalPositionT) {

	if (typeof robot.joints[currentJoint] === 'undefined') {

		/*
		If the current joint is a link and is not the base, directly go to its parent joint
		*/
		if (robot.links[currentJoint].name !== robot.base ) {

			generateJacobianMatrix(robot.links[currentJoint].parent, jacobianMatrix, endEffectorGlobalPositionT);

		}

	} else {

		/*
		Get the current joint's local position
		*/
		var jointLocalPosition = [];
		for (var i = 0; i < 3; i++) {

			jointLocalPosition[i] = [0];

		}
		jointLocalPosition[3] = [1];

		/*
		Transform the joint's local position into global position
		*/
		var jointGlobalPosition = matrix_multiply(robot.joints[currentJoint].xform, jointLocalPosition);

		/*
		Transpose of the joint's global position
		*/
		var jointGlobalPositionT = [];
		for (var i = 0; i < 3; i++) {

			jointGlobalPositionT[i] = jointGlobalPosition[i][0];

		}

		/*
		Get the current joint's local axices
		*/
		var jointLocalAxis = [];
		for (var i = 0; i < 3; i++) {

			jointLocalAxis[i] = [robot.joints[currentJoint].axis[i]];

		}
		jointLocalAxis[3] = [1];

		/*
		Transform the joint's local axices into axices in global view
		*/
		var jointGlobalAxis = matrix_multiply(robot.joints[currentJoint].xform, jointLocalAxis);

		/*
		Transpose of the joint's global axices
		*/
		var jointGlobalAxisT = [];
		for (var i = 0; i < 3; i++) {

			jointGlobalAxisT[i] = jointGlobalAxis[i][0];

		}

		/*
		The position difference between the current joint and the end effector
		*/
		var jointEndEffectorPositionDifferenceT = [];
		for (var i = 0; i < 3; i++) {

			jointEndEffectorPositionDifferenceT[i] = endEffectorGlobalPositionT[i] - jointGlobalPositionT[i];

		}

		/*
		The upper three rows of the jacobian matrix
		*/
		var jacobianUpperRowsT = vector_cross(jointGlobalAxisT, jointEndEffectorPositionDifferenceT);

		for (var i = 0; i < 3; i++) {

			jacobianMatrix[i].unshift(jacobianUpperRowsT[i]);

		}

		/*
		The lower three rows of the jacobian matrix which is the current joint's axices in global view
		*/
		for (var i = 0; i < 3; i++) {

			jacobianMatrix[i + 3].unshift(jointGlobalAxisT[i]);

		}

		generateJacobianMatrix(robot.joints[currentJoint].parent, jacobianMatrix, endEffectorGlobalPositionT);

	}

}

/*
Recursively assign the angular displacement to each joint
*/
function renderAngularDisplacement(currentJoint, angularDisplacement) {

	if (typeof robot.joints[currentJoint] === 'undefined') {

		/*
		If the current joint is a link and is not the base, directly go to its parent joint
		*/
		if (robot.links[currentJoint].name !== robot.base ) {

			renderAngularDisplacement(robot.links[currentJoint].parent, angularDisplacement);

		}

	} else { 

		/*
		Assign the angular displacement
		*/
		robot.joints[currentJoint].control = 0.1 * (angularDisplacement.pop())[0];

		renderAngularDisplacement(robot.joints[currentJoint].parent, angularDisplacement);

	}

}