//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:
*/

/*
Matrix multiplication
*/
function matrix_multiply(matrixA, matrixB) {
	
	var productMatrix = [];
	var rowNum = matrixA.length;
	var columNum = matrixB[0].length;
	
	var m = matrixA[0].length;
	
	for(var i = 0; i < rowNum; i++) {
		
		productMatrix[i] = [];
		
		for(var j = 0; j < columNum; j++) {
			
			var entry = 0;
			
			for(var k = 0; k < m; k++) {
				
				entry += matrixA[i][k] * matrixB[k][j];
				
			}
			
			productMatrix[i][j] = entry;
			
		}
		
	}
	
	return productMatrix;
	
}

/*
Matrix transpose
*/
function matrix_transpose(matrix) {
	
	var transposeMatrix = [];
	
	for(var i = 0; i < matrix[0].length; i++) {
		
		transposeMatrix[i] = [];
		
		for(var j = 0; j < matrix.length; j++) {	
				
				transposeMatrix[i][j] = matrix[j][i]
			
		}
	
	}
	
	return transposeMatrix;

}

/*
matrix_pseudoinverse (assuming numeric.inv)
*/

/*
Normalize a vector
*/
function vector_normalize(vector) {
	
	var normalizedVector = [];
	
	var magnitude = 0;
	
	for(var i = 0; i < vector.length; i++) {
		
		magnitude += vector[i] * vector[i];
		
	}
	
	magnitude = Math.sqrt(magnitude);
	
	for(var i = 0; i < vector.length; i++) {

		normalizedVector[i] = vector[i] / magnitude;
		
	}
	
	return normalizedVector;
	
}

/*
The cross product of two vectors
*/
function vector_cross(vectorA, vectorB) {
	
	var crossProduct = [];
	
	crossProduct[0] = vectorA[1] * vectorB[2] - vectorA[2] * vectorB[1];
	crossProduct[1] = vectorA[2] * vectorB[0] - vectorA[0] * vectorB[2];
	crossProduct[2] = vectorA[0] * vectorB[1] - vectorA[1] * vectorB[0];
	
	return crossProduct;
	
}

/*
Generate identity matrix
*/
function generate_identity() {
	
	var identityMatrix = [
							[1,0,0,0],
							[0,1,0,0],
							[0,0,1,0],
							[0,0,0,1],
						];
						
	return identityMatrix;

}

/*
Generate translation matrix based on local position
*/
function generate_translation_matrix(x, y, z) {
	
	var translation_matrix = generate_identity();
	
	translation_matrix[0][3] = x;
	translation_matrix[1][3] = y;
	translation_matrix[2][3] = z;
	
	return translation_matrix;
	
}

/*
Generate rotation matrix according to x demension
*/
function generate_rotation_matrix_X(theta) {
	
	var rotationMatrix = generate_identity();

	rotationMatrix[1][1] = Math.cos(theta);
	rotationMatrix[1][2] = -Math.sin(theta);
	rotationMatrix[2][1] = Math.sin(theta);
	rotationMatrix[2][2] = Math.cos(theta);
	
	return rotationMatrix;
	
}

/*
Generate rotation matrix according to y demension
*/
function generate_rotation_matrix_Y(theta) {
	
	var rotationMatrix = generate_identity();

	rotationMatrix[0][0] = Math.cos(theta);
	rotationMatrix[0][2] = Math.sin(theta);
	rotationMatrix[2][0] = -Math.sin(theta);
	rotationMatrix[2][2] = Math.cos(theta);
	
	return rotationMatrix;
	
}

/*
Generate rotation matrix according to z demension
*/
function generate_rotation_matrix_Z(theta) {
	
	var rotationMatrix = generate_identity();

	rotationMatrix[0][0] = Math.cos(theta);
	rotationMatrix[0][1] = -Math.sin(theta);
	rotationMatrix[1][0] = Math.sin(theta);
	rotationMatrix[1][1] = Math.cos(theta);
	
	return rotationMatrix
	
}