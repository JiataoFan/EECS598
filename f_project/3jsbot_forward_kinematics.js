//////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:
*/
function robot_forward_kinematics() {

	traverse_forward_kinematics_link(robot.base);
	
}

function traverse_forward_kinematics_link(currentLink) {

	/*
	Base of the robot
	*/
	 if (typeof robot.links[currentLink].parent === 'undefined') {

		var translationMatrix = generate_translation_matrix(robot.origin.xyz[0], 
															robot.origin.xyz[1], 
															robot.origin.xyz[2]);

		var rRotationMatrix = generate_rotation_matrix_X(robot.origin.rpy[0]);
		var pRotationMatrix = generate_rotation_matrix_Y(robot.origin.rpy[1]);
		var yRotationMatrix = generate_rotation_matrix_Z(robot.origin.rpy[2]);
	
		var transformMatrix = matrix_multiply(translationMatrix, rRotationMatrix);
		transformMatrix = matrix_multiply(transformMatrix, pRotationMatrix);
		transformMatrix = matrix_multiply(transformMatrix, yRotationMatrix);
	
		robot.links[currentLink].xform = transformMatrix;
		
	} else {
		
		robot.links[currentLink].xform = robot.joints[robot.links[currentLink].parent].xform;
		
	}
	
	var tempmat = matrix_2Darray_to_threejs(robot.links[currentLink].xform);
	simpleApplyMatrix(robot.links[currentLink].geom, tempmat);
	
	if (typeof robot.links[currentLink].children !== 'undefined') {
	
		for(var i = 0; i < robot.links[currentLink].children.length; i++){
		
			traverse_forward_kinematics_joint(robot.links[currentLink].children[i]);
		
		}
	
	}
	
}

function traverse_forward_kinematics_joint(currentJoint) {

	var translationMatrix = generate_translation_matrix(robot.joints[currentJoint].origin.xyz[0], 
														robot.joints[currentJoint].origin.xyz[1], 
														robot.joints[currentJoint].origin.xyz[2]);

	var rRotationMatrix = generate_rotation_matrix_X(robot.joints[currentJoint].origin.rpy[0]);
	var pRotationMatrix = generate_rotation_matrix_Y(robot.joints[currentJoint].origin.rpy[1]);
	var yRotationMatrix = generate_rotation_matrix_Z(robot.joints[currentJoint].origin.rpy[2]);
	
	var motorRotationMatrix = quaternion_to_rotation_matrix(quaternion_normalize(quaternion_from_axisangle(currentJoint)));
	
	var transformMatrix = matrix_multiply(translationMatrix, rRotationMatrix);
	transformMatrix = matrix_multiply(transformMatrix, pRotationMatrix);
	transformMatrix = matrix_multiply(transformMatrix, yRotationMatrix);
	transformMatrix = matrix_multiply(transformMatrix, motorRotationMatrix);
	transformMatrix = matrix_multiply(robot.links[robot.joints[currentJoint].parent].xform,transformMatrix);
	
	robot.joints[currentJoint].origin.xform = transformMatrix;
	robot.joints[currentJoint].xform = robot.joints[currentJoint].origin.xform;

	var tempmat = matrix_2Darray_to_threejs(robot.joints[currentJoint].xform);
	simpleApplyMatrix(robot.joints[currentJoint].geom, tempmat);

	traverse_forward_kinematics_link(robot.joints[currentJoint].child);

}

/*
compute_and_draw_heading
*/