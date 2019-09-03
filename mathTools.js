function solveAssignmentProblem1(costMatrix){
	var N = costMatrix.length;
	
	
	var costMatrixV2 = costMatrix;//row reduction
	for(var rowID = 0; rowID<N; rowID++){
		var rowMin = min(costMatrix[rowID]); 
		//print(rowMin);
		for(var colID = 0; colID<N; colID++){
			costMatrixV2[rowID][colID] = costMatrix[rowID][colID]-rowMin; 
		}
	}
	
	var costMatrixV2T = []; //getting transpose of row reduced matrix
	for(var rowID = 0; rowID<N; rowID++){
		append(costMatrixV2T,[]);
		for(var colID = 0; colID<N; colID++){
			costMatrixV2T[rowID][colID] = costMatrixV2[colID][rowID]; 
		}
	}
	
	
	var costMatrixV3T = costMatrixV2T;//coloumn reduction
	for(var rowID = 0; rowID<N; rowID++){
		var rowMin = min(costMatrixV2T[rowID]); 
		//print(rowMin );
		for(var colID = 0; colID<N; colID++){
			costMatrixV3T[rowID][colID] = costMatrixV2T[rowID][colID]-rowMin; 
		}
	}
	
	
	var costMatrixV3 = []; //getting transpose of coloumn reduced matrix
	for(var rowID = 0; rowID<N; rowID++){ 
		append(costMatrixV3,[]);
		for(var colID = 0; colID<N; colID++){
			costMatrixV3[rowID][colID] = costMatrixV3T[colID][rowID]; 
		}
	}
	
	
	//iterative algorithm
	var optimalityReached=false;
	var assignProbIteration = 0;
	while(!optimalityReached){
		assignProbIteration++;
		
		print("Mat");
		print(costMatrixV3);
		
		
		
		
		if(assignProbIteration>50){
			print("optimality Not Reached");
			/*print("Mat");
			print(costMatrixV3);
			print("iteration");
			print(assignProbIteration);
			print("sqzeros");
			print(squaredZeros);
			print("remains");
			print(remainingRows);
			print(remainingCols);
			print("marked");
			print(markedRows);
			print(markedCols);*/
			return Array.apply(null, {length: N}).map(Number.call, Number);
			
			
		}
		
		
		//step1
		var squaredZeros=[];
		var remainingCols=Array.apply(null, {length: N}).map(Number.call, Number);
		var remainingRows=Array.apply(null, {length: N}).map(Number.call, Number);
		var markedRows = [];
		var markedCols = [];
		
		
		var totalRemainingZerosCount = 1; //for initial row and colomn scanningstart
				
		while(totalRemainingZerosCount==1){
			//row scan on costMatrixV3
			for(var rowID = 0; rowID<remainingRows.length; rowID++){
				var rowIDSearch = remainingRows[rowID];
				var zerosCount=0;
				var tempColIndex;
				for(var colID = 0; colID<remainingCols.length; colID++){
					var colIDSearch = remainingCols[colID];
					if(costMatrixV3[rowIDSearch][colIDSearch]==0){
						zerosCount++;
						tempColIndex=colIDSearch;
					} 
				}
				if(zerosCount==1){
					append(squaredZeros,[rowIDSearch,tempColIndex]);
					var indexToBeRemoved = remainingCols.indexOf(tempColIndex);
					remainingCols.splice(indexToBeRemoved, 1);
					append(markedCols,tempColIndex);
				}
			}
			//column scan on costMatrixV3
			for(var colID = 0; colID<remainingCols.length; colID++){
				var colIDSearch = remainingCols[colID];
				var zerosCount=0;
				var tempRowIndex;
				for(var rowID = 0; rowID<remainingRows.length; rowID++){
					var rowIDSearch = remainingRows[rowID];
					if(costMatrixV3[rowIDSearch][colIDSearch]==0){
						zerosCount++;
						tempRowIndex=rowIDSearch;
					} 
				}
				if(zerosCount==1){
					append(squaredZeros,[tempRowIndex,colIDSearch]);
					var indexToBeRemoved = remainingRows.indexOf(tempRowIndex);
					remainingRows.splice(indexToBeRemoved, 1);
					append(markedRows,tempRowIndex);
				}
			}
			
			//check whether remaining total uncut zeros, if 1: loop continue, if zero: loop break, if >=2: diagonal cut
			totalRemainingZerosCount = 0;
			var remainingZerosCoordinates = [];
			for(var rowID=0;rowID<remainingRows.length;rowID++){
				var rowIDSearch = remainingRows[rowID];
				for(var colID=0;colID<remainingCols.length;colID++){
					var colIDSearch = remainingCols[colID];
					if(costMatrixV3[rowIDSearch][colIDSearch]==0){
						totalRemainingZerosCount++;
						append(remainingZerosCoordinates,[rowIDSearch,colIDSearch]);
					}
				}
			}
			
			
			if(totalRemainingZerosCount==0){
				break;
			}else if(totalRemainingZerosCount>=2){
				print("multiple uncut zeros exist - more than one optimal sol");
				
				//identify diagonally placed uncut zeros
				var avoidRows = [];//diagonal zero cancelling
				var avoidCols = [];
				append(squaredZeros,remainingZerosCoordinates[0]);
				
				//cut column, avoid row
				//or cut row avoid column
				append(squaredZeros,remainingZerosCoordinates[0]);
				append(avoidRows,remainingZerosCoordinates[0][0]);
				
				if(avoidRows.length>=1){
					for(var zeroID=0; zeroID<totalRemainingZerosCount; zeroID++){
						if(remainingZerosCoordinates[zeroID][0]!=avoidRows[avoidRows.length-1]){//diagonally placed zero
							
						}
					}
				}else{
					
					var indexToBeRemoved = remainingCols.indexOf(remainingZerosCoordinates[0][1]);
					remainingCols.splice(indexToBeRemoved, 1);
					append(markedCols,remainingZerosCoordinates[0][1]);
				}
				
				//var indexToBeRemoved = remainingRows.indexOf(remainingZerosCoordinates[0][0]);
				//remainingRows.splice(indexToBeRemoved, 1);
				//append(markedRows,remainingZerosCoordinates[0][0]);
				
				print(remainingZerosCoordinates);
				
			}
			
			
		}
		//if zeros are there  even after row and colomn scaning: https://youtu.be/-0DEQmp7B9o : min19.53
		
		
		
		
		
		
		
		
		//step2
		if(squaredZeros.length!=N){
			optimalityReached = false;
			
			//step3
			
			//for undeleted entries
			var remainingCells = [];
			for(var rowID=0;rowID<remainingRows.length;rowID++){
				var rowIDSearch = remainingRows[rowID];
				for(var colID=0;colID<remainingCols.length;colID++){
					var colIDSearch = remainingCols[colID];
					if(costMatrixV3[rowIDSearch][colIDSearch]==0){
						print("bloody trouble!!!");
					}
					append(remainingCells,costMatrixV3[rowIDSearch][colIDSearch]);
				}
			}
			
			var minRemainingCellValue = min(remainingCells); 
			for(var rowID=0;rowID<remainingRows.length;rowID++){
				var rowIDSearch = remainingRows[rowID];
				for(var colID=0;colID<remainingCols.length;colID++){
					var colIDSearch = remainingCols[colID];
					costMatrixV3[rowIDSearch][colIDSearch] = costMatrixV3[rowIDSearch][colIDSearch]-minRemainingCellValue;
				} 
			}
			
			//for crossed elements
			for(var rowID=0;rowID<markedRows.length;rowID++){
				var rowIDSearch = markedRows[rowID];
				for(var colID=0;colID<markedCols.length;colID++){
					var colIDSearch = markedCols[colID];
					costMatrixV3[rowIDSearch][colIDSearch] = costMatrixV3[rowIDSearch][colIDSearch]+minRemainingCellValue;
				} 
			}
			
			
		}
		else{
			optimalityReached = true;
			result=Array.apply(null, {length: N}).map(Number.call, Number);//ith element=who is going to take ith job
			for(var i=0;i<N;i++){
				result[squaredZeros[i][0]]=squaredZeros[i][1];
			}
			return result
			//out the results using squaredZeros
		}
		
		print("iteration");
		print(assignProbIteration);
		print("sqzeros");
		print(squaredZeros);
		print("remains");
		print(remainingRows);
		print(remainingCols);
		print("marked");
		print(markedRows);
		print(markedCols);
		
		
		/*print("Mat");
		print(costMatrixV3);
		print("iteration");
		print(assignProbIteration);
		print("sqzeros");
		print(squaredZeros);
		print("remains");
		print(remainingRows);
		print(remainingCols);
		print("marked");
		print(markedRows);
		print(markedCols);*/
	}

	//return costMatrixV3;
	
	
}

//costMatrix = [[9,11,14,11,7],[6,15,13,13,10],[12,13,6,8,8],[11,9,10,12,9],[7,12,14,10,14]];
//costMatrix = [[90,75,75,80],[35,85,55,65],[125,95,90,105],[45,110,95,115]];
//costMatrix = [[250,400,350],[400,600,350],[200,400,250]];

/*costMatrix =  [[27.227603134868946, 249.8025139467172, 466.91115236972763, 314.2975840552035, 151.38640699227054, 375.61763971292135, 350.5228243785919], 
	[248.94542292990332, 57.68460589312442, 226.1730017779681, 236.63285174987863, 203.74126988184474, 175.06884985825025, 276.53249466138675],
	[458.6561955671582, 220.60827746711342, 38.56499383082747, 349.61507571520724, 383.48056907590393, 154.92690409453974, 346.43854846242135],
	[286.99684796519443, 156.45727229896923, 320.28799254192415, 23.597012433952724, 363.0204093331636, 374.30713738464635, 486.68727266069146],
	[286.05596012810565, 270.95027198613855, 371.39142838758636, 438.67671307123317, 112.20109993312073, 195.02081225305824, 89.4920883693505],
	[412.9748089806102, 358.5425848745763, 386.17135452417443, 536.5916669209697, 238.3640042529678, 194.02027088916617, 38.21065305562695],
	[431.6334268596361, 395.31732531096065, 427.1613537310061, 572.0305353662685, 254.592415355589, 235.17777425211787, 62.49660117020377]];*/

/*costMatrix = [[210, 446, 446, 595],
	[196, 433, 433, 580],
	[182, 421, 420, 566],
	[168, 408, 408, 552]];*/

/*costMatrix = [[210.97811537678214, 446.45858759544024, 446.33321824698595, 595.0469871743263],
	[196.8359852556315, 433.7360572429742, 433.6074760850126, 580.9048515584799],
	[182.69385598637837, 421.10410146143886, 420.97214269226646, 566.7627159430269],
	[168.5517277834551, 408.5711213272555, 408.4356087916688, 552.6205803279976]];*/


/*costMatrix = [[53, 312, 259, 152, 341, 409],[301, 67, 396, 138, 328, 253],[315, 424, 75, 235, 122, 302],[427, 283, 335, 230, 161, 40],[585, 415, 449, 390, 252, 134],[571, 403, 437, 376, 240, 120]];*/


/*costMatrix = [[37, 354, 157, 170, 348, 372, 358, 502],[353, 28, 369, 166, 497, 167, 361, 359],[323, 483, 134, 339, 58, 370, 183, 374],[235, 231, 171, 109, 299, 174, 203, 306],[429, 399, 265, 337, 210, 225, 36, 162],[505, 328, 387, 355, 380, 141, 195, 52],[585, 412, 452, 442, 411, 229, 236, 49],[571, 401, 439, 428, 400, 217, 224, 35]];
*/

/*costMatrix = [[232, 446, 576],[218, 433, 561],[204, 420, 547]]*/

























var MAX_SIZE = parseInt(Number.MAX_SAFE_INTEGER/2) || ((1 << 26)*(1 << 26));

/**
 * A default value to pad the cost matrix with if it is not quadratic.
 */
var DEFAULT_PAD_VALUE = 0;

// ---------------------------------------------------------------------------
// Classes
// ---------------------------------------------------------------------------

/**
 * Calculate the Munkres solution to the classical assignment problem.
 * See the module documentation for usage.
 * @constructor
 */
function Munkres() {
  this.C = null;

  this.row_covered = [];
  this.col_covered = [];
  this.n = 0;
  this.Z0_r = 0;
  this.Z0_c = 0;
  this.marked = null;
  this.path = null;
}

/**
 * Pad a possibly non-square matrix to make it square.
 *
 * @param {Array} matrix An array of arrays containing the matrix cells
 * @param {Number} [pad_value] The value used to pad a rectangular matrix
 *
 * @return {Array} An array of arrays representing the padded matrix
 */
Munkres.prototype.pad_matrix = function(matrix, pad_value) {
  pad_value = pad_value || DEFAULT_PAD_VALUE;

  var max_columns = 0;
  var total_rows = matrix.length;
  var i;

  for (i = 0; i < total_rows; ++i)
    if (matrix[i].length > max_columns)
      max_columns = matrix[i].length;

  total_rows = max_columns > total_rows ? max_columns : total_rows;

  var new_matrix = [];

  for (i = 0; i < total_rows; ++i) {
    var row = matrix[i] || [];
    var new_row = row.slice();

    // If this row is too short, pad it
    while (total_rows > new_row.length)
      new_row.push(pad_value);

    new_matrix.push(new_row);
  }

  return new_matrix;
};

/**
 * Compute the indices for the lowest-cost pairings between rows and columns
 * in the database. Returns a list of (row, column) tuples that can be used
 * to traverse the matrix.
 *
 * **WARNING**: This code handles square and rectangular matrices.
 * It does *not* handle irregular matrices.
 *
 * @param {Array} cost_matrix The cost matrix. If this cost matrix is not square,
 *                            it will be padded with DEFAULT_PAD_VALUE. Optionally,
 *                            the pad value can be specified via options.padValue.
 *                            This method does *not* modify the caller's matrix.
 *                            It operates on a copy of the matrix.
 * @param {Object} [options] Additional options to pass in
 * @param {Number} [options.padValue] The value to use to pad a rectangular cost_matrix
 *
 * @return {Array} An array of ``(row, column)`` arrays that describe the lowest
 *                 cost path through the matrix
 */
Munkres.prototype.compute = function(cost_matrix, options) {

  options = options || {};
  options.padValue = options.padValue || DEFAULT_PAD_VALUE;

  this.C = this.pad_matrix(cost_matrix, options.padValue);
  this.n = this.C.length;
  this.original_length = cost_matrix.length;
  this.original_width = cost_matrix[0].length;

  var nfalseArray = []; /* array of n false values */
  while (nfalseArray.length < this.n)
    nfalseArray.push(false);
  this.row_covered = nfalseArray.slice();
  this.col_covered = nfalseArray.slice();
  this.Z0_r = 0;
  this.Z0_c = 0;
  this.path =   this.__make_matrix(this.n * 2, 0);
  this.marked = this.__make_matrix(this.n, 0);

  var step = 1;

  var steps = { 1 : this.__step1,
                2 : this.__step2,
                3 : this.__step3,
                4 : this.__step4,
                5 : this.__step5,
                6 : this.__step6 };

  while (true) {
    var func = steps[step];
    if (!func) // done
      break;

    step = func.apply(this);
  }

  var results = [];
  for (var i = 0; i < this.original_length; ++i)
    for (var j = 0; j < this.original_width; ++j)
      if (this.marked[i][j] == 1)
        results.push([i, j]);

  return results;
};

/**
 * Create an n×n matrix, populating it with the specific value.
 *
 * @param {Number} n Matrix dimensions
 * @param {Number} val Value to populate the matrix with
 *
 * @return {Array} An array of arrays representing the newly created matrix
 */
Munkres.prototype.__make_matrix = function(n, val) {
  var matrix = [];
  for (var i = 0; i < n; ++i) {
    matrix[i] = [];
    for (var j = 0; j < n; ++j)
      matrix[i][j] = val;
  }

  return matrix;
};

/**
 * For each row of the matrix, find the smallest element and
 * subtract it from every element in its row. Go to Step 2.
 */
Munkres.prototype.__step1 = function() {
  for (var i = 0; i < this.n; ++i) {
    // Find the minimum value for this row and subtract that minimum
    // from every element in the row.
    var minval = Math.min.apply(Math, this.C[i]);

    for (var j = 0; j < this.n; ++j)
      this.C[i][j] -= minval;
  }

  return 2;
};

/**
 * Find a zero (Z) in the resulting matrix. If there is no starred
 * zero in its row or column, star Z. Repeat for each element in the
 * matrix. Go to Step 3.
 */
Munkres.prototype.__step2 = function() {
  for (var i = 0; i < this.n; ++i) {
    for (var j = 0; j < this.n; ++j) {
      if (this.C[i][j] === 0 &&
        !this.col_covered[j] &&
        !this.row_covered[i])
      {
        this.marked[i][j] = 1;
        this.col_covered[j] = true;
        this.row_covered[i] = true;
        break;
      }
    }
  }

  this.__clear_covers();

  return 3;
};

/**
 * Cover each column containing a starred zero. If K columns are
 * covered, the starred zeros describe a complete set of unique
 * assignments. In this case, Go to DONE, otherwise, Go to Step 4.
 */
Munkres.prototype.__step3 = function() {
  var count = 0;

  for (var i = 0; i < this.n; ++i) {
    for (var j = 0; j < this.n; ++j) {
      if (this.marked[i][j] == 1 && this.col_covered[j] == false) {
        this.col_covered[j] = true;
        ++count;
      }
    }
  }

  return (count >= this.n) ? 7 : 4;
};

/**
 * Find a noncovered zero and prime it. If there is no starred zero
 * in the row containing this primed zero, Go to Step 5. Otherwise,
 * cover this row and uncover the column containing the starred
 * zero. Continue in this manner until there are no uncovered zeros
 * left. Save the smallest uncovered value and Go to Step 6.
 */

Munkres.prototype.__step4 = function() {
  var done = false;
  var row = -1, col = -1, star_col = -1;

  while (!done) {
    var z = this.__find_a_zero();
    row = z[0];
    col = z[1];

    if (row < 0)
      return 6;

    this.marked[row][col] = 2;
    star_col = this.__find_star_in_row(row);
    if (star_col >= 0) {
      col = star_col;
      this.row_covered[row] = true;
      this.col_covered[col] = false;
    } else {
      this.Z0_r = row;
      this.Z0_c = col;
      return 5;
    }
  }
};

/**
 * Construct a series of alternating primed and starred zeros as
 * follows. Let Z0 represent the uncovered primed zero found in Step 4.
 * Let Z1 denote the starred zero in the column of Z0 (if any).
 * Let Z2 denote the primed zero in the row of Z1 (there will always
 * be one). Continue until the series terminates at a primed zero
 * that has no starred zero in its column. Unstar each starred zero
 * of the series, star each primed zero of the series, erase all
 * primes and uncover every line in the matrix. Return to Step 3
 */
Munkres.prototype.__step5 = function() {
  var count = 0;

  this.path[count][0] = this.Z0_r;
  this.path[count][1] = this.Z0_c;
  var done = false;

  while (!done) {
    var row = this.__find_star_in_col(this.path[count][1]);
    if (row >= 0) {
      count++;
      this.path[count][0] = row;
      this.path[count][1] = this.path[count-1][1];
    } else {
      done = true;
    }

    if (!done) {
      var col = this.__find_prime_in_row(this.path[count][0]);
      count++;
      this.path[count][0] = this.path[count-1][0];
      this.path[count][1] = col;
    }
  }

  this.__convert_path(this.path, count);
  this.__clear_covers();
  this.__erase_primes();
  return 3;
};

/**
 * Add the value found in Step 4 to every element of each covered
 * row, and subtract it from every element of each uncovered column.
 * Return to Step 4 without altering any stars, primes, or covered
 * lines.
 */
Munkres.prototype.__step6 = function() {
  var minval = this.__find_smallest();

  for (var i = 0; i < this.n; ++i) {
    for (var j = 0; j < this.n; ++j) {
      if (this.row_covered[i])
        this.C[i][j] += minval;
      if (!this.col_covered[j])
        this.C[i][j] -= minval;
    }
  }

  return 4;
};

/**
 * Find the smallest uncovered value in the matrix.
 *
 * @return {Number} The smallest uncovered value, or MAX_SIZE if no value was found
 */
Munkres.prototype.__find_smallest = function() {
  var minval = MAX_SIZE;

  for (var i = 0; i < this.n; ++i)
    for (var j = 0; j < this.n; ++j)
      if (!this.row_covered[i] && !this.col_covered[j])
        if (minval > this.C[i][j])
          minval = this.C[i][j];

  return minval;
};

/**
 * Find the first uncovered element with value 0.
 *
 * @return {Array} The indices of the found element or [-1, -1] if not found
 */
Munkres.prototype.__find_a_zero = function() {
  for (var i = 0; i < this.n; ++i)
    for (var j = 0; j < this.n; ++j)
      if (this.C[i][j] === 0 &&
        !this.row_covered[i] &&
        !this.col_covered[j])
        return [i, j];

  return [-1, -1];
};

/**
 * Find the first starred element in the specified row. Returns
 * the column index, or -1 if no starred element was found.
 *
 * @param {Number} row The index of the row to search
 * @return {Number}
 */

Munkres.prototype.__find_star_in_row = function(row) {
  for (var j = 0; j < this.n; ++j)
    if (this.marked[row][j] == 1)
      return j;

  return -1;
};

/**
 * Find the first starred element in the specified column.
 *
 * @return {Number} The row index, or -1 if no starred element was found
 */
Munkres.prototype.__find_star_in_col = function(col) {
  for (var i = 0; i < this.n; ++i)
    if (this.marked[i][col] == 1)
      return i;

  return -1;
};

/**
 * Find the first prime element in the specified row.
 *
 * @return {Number} The column index, or -1 if no prime element was found
 */

Munkres.prototype.__find_prime_in_row = function(row) {
  for (var j = 0; j < this.n; ++j)
    if (this.marked[row][j] == 2)
      return j;

  return -1;
};

Munkres.prototype.__convert_path = function(path, count) {
  for (var i = 0; i <= count; ++i)
    this.marked[path[i][0]][path[i][1]] =
      (this.marked[path[i][0]][path[i][1]] == 1) ? 0 : 1;
};

/** Clear all covered matrix cells */
Munkres.prototype.__clear_covers = function() {
  for (var i = 0; i < this.n; ++i) {
    this.row_covered[i] = false;
    this.col_covered[i] = false;
  }
};

/** Erase all prime markings */
Munkres.prototype.__erase_primes = function() {
  for (var i = 0; i < this.n; ++i)
    for (var j = 0; j < this.n; ++j)
      if (this.marked[i][j] == 2)
        this.marked[i][j] = 0;
};

// ---------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------

/**
 * Create a cost matrix from a profit matrix by calling
 * 'inversion_function' to invert each value. The inversion
 * function must take one numeric argument (of any type) and return
 * another numeric argument which is presumed to be the cost inverse
 * of the original profit.
 *
 * This is a static method. Call it like this:
 *
 *  cost_matrix = make_cost_matrix(matrix[, inversion_func]);
 *
 * For example:
 *
 *  cost_matrix = make_cost_matrix(matrix, function(x) { return MAXIMUM - x; });
 *
 * @param {Array} profit_matrix An array of arrays representing the matrix
 *                              to convert from a profit to a cost matrix
 * @param {Function} [inversion_function] The function to use to invert each
 *                                       entry in the profit matrix
 *
 * @return {Array} The converted matrix
 */
function make_cost_matrix (profit_matrix, inversion_function) {
  var i, j;
  if (!inversion_function) {
    var maximum = -1.0/0.0;
    for (i = 0; i < profit_matrix.length; ++i)
      for (j = 0; j < profit_matrix[i].length; ++j)
        if (profit_matrix[i][j] > maximum)
          maximum = profit_matrix[i][j];

    inversion_function = function(x) { return maximum - x; };
  }

  var cost_matrix = [];

  for (i = 0; i < profit_matrix.length; ++i) {
    var row = profit_matrix[i];
    cost_matrix[i] = [];

    for (j = 0; j < row.length; ++j)
      cost_matrix[i][j] = inversion_function(profit_matrix[i][j]);
  }

  return cost_matrix;
}

/**
 * Convenience function: Converts the contents of a matrix of integers
 * to a printable string.
 *
 * @param {Array} matrix The matrix to print
 *
 * @return {String} The formatted matrix
 */
function format_matrix(matrix) {
  var columnWidths = [];
  var i, j;
  for (i = 0; i < matrix.length; ++i) {
    for (j = 0; j < matrix[i].length; ++j) {
      var entryWidth = String(matrix[i][j]).length;

      if (!columnWidths[j] || entryWidth >= columnWidths[j])
        columnWidths[j] = entryWidth;
    }
  }

  var formatted = '';
  for (i = 0; i < matrix.length; ++i) {
    for (j = 0; j < matrix[i].length; ++j) {
      var s = String(matrix[i][j]);

      // pad at front with spaces
      while (s.length < columnWidths[j])
        s = ' ' + s;

      formatted += s;

      // separate columns
      if (j != matrix[i].length - 1)
        formatted += ' ';
    }

    if (i != matrix[i].length - 1)
      formatted += '\n';
  }

  return formatted;
}

// ---------------------------------------------------------------------------
// Exports
// ---------------------------------------------------------------------------
//solveAssignmentProblem
function solveAssignmentProblem(cost_matrix, options) {
  var m = new Munkres();
  return m.compute(cost_matrix, options);
}

solveAssignmentProblem.version = "1.2.2";
solveAssignmentProblem.format_matrix = format_matrix;
solveAssignmentProblem.make_cost_matrix = make_cost_matrix;
solveAssignmentProblem.Munkres = Munkres; // backwards compatibility

if (typeof module !== 'undefined' && module.exports) {
  module.exports = solveAssignmentProblem;
}



function solveSimplexTableu(simplexTableu){

	//print(simplexTableu);
	var A = simplexTableu; //stored
	print(A);
	numberOfEdges = math.size(simplexTableu)[1]-1;
	numberOfNodes = math.size(simplexTableu)[0]-1;
	simplexTableu.pop(); //removed the last row - redundent
	
	
	// dual simplex method
	print(simplexTableu);

	//initial transform
	simplexMatrix = math.matrix(simplexTableu);
	
	
	print(simplexMatrix)

	//pivoting - initial
	var lastPickedColumnIndex = 1;//columns that were picked already
	var pickedColumnIndexes = [];//columns that were picked already
	//var scanningRowIndex = 1; //till numberOfNodes

	for(var scanningRowIndex = 1; scanningRowIndex < (numberOfNodes); scanningRowIndex++){
		for(var scanningColumnIndex = lastPickedColumnIndex; scanningColumnIndex < numberOfEdges; scanningColumnIndex++) {
			if(simplexMatrix._data[scanningRowIndex][scanningColumnIndex]!=0){
				print("Pivot "+scanningRowIndex+" and "+scanningColumnIndex);
				//simplexMatrix = pivotElement(simplexMatrix,scanningRowIndex,scanningColumnIndex);
				//print(simplexMatrix);
				pickedColumnIndexes.push(scanningColumnIndex);
				if(pickedColumnIndexes.length < (numberOfNodes-particleShadows.length)){
					lastPickedColumnIndex = scanningColumnIndex+1;
					break;
				}else{
					for(var i = particleShadows.length-1; i>0; i--){
						pickedColumnIndexes.push(numberOfEdges-i+1);
						
					}
					scanningRowIndex = (numberOfNodes);//finish sencond loop
					break;
				}
			}
		}
	}

	print("Selected Basis Columns");
	print(pickedColumnIndexes);//these links = initial basis
	
	if(pickedColumnIndexes.length!=(numberOfNodes-1)){
		print("Initial BFS Not Found");
	}

	var basisPivots = [];
	for(var i=0;i<pickedColumnIndexes.length;i++){
		simplexMatrix = pivotElement(simplexMatrix,i+1,pickedColumnIndexes[i]);
		basisPivots.push([i+1,pickedColumnIndexes[i]]);//row and col index of pivot
	}

	print(simplexMatrix); // initial simplex tableu
	//
	consolePrint("Solving Optimal Routing Problem...")
	// simplex iterations
	for(var i = 1; i<2000; i++){

		if(checkFeasibility(simplexMatrix)){
			print("Continue to Iteration "+i);
		}
		else{
			print("Infeasible !!! at Itration "+i);
			break;
		}

		
		// finding pivot column
		var pivotColumnIndex;
		var reducedCosts = simplexMatrix._data[0];
		for(var j = 1; j < reducedCosts.length; j++){
			if(reducedCosts[j]<0){
				pivotColumnIndex = j;
				break;
			}
		}
		print("Cj: "+reducedCosts[pivotColumnIndex]+" at "+pivotColumnIndex);


		//finding pivot row
		var pivotRowIndex = -1;
		var pivotRowRatios = [];
		
		var pivotColumn = simplexMatrix.subset(math.index(math.range(1,numberOfNodes),[pivotColumnIndex]))._data;
		var solutionColumn = simplexMatrix.subset(math.index(math.range(1,numberOfNodes),[0]))._data;
		print("Pivot Column");
		print(pivotColumn);
		print("Solution Column");
		print(solutionColumn);
		var positiveCount = 0;
		var tempMinimum  = 1000000000;
		for(var j=0; j < (numberOfNodes-1); j++ ){
			if(pivotColumn[j]>0){
				var pivotRowRatio = solutionColumn[j]/pivotColumn[j];
				pivotRowRatios.push(pivotRowRatio);
				if(tempMinimum>pivotRowRatio){
					tempMinimum = pivotRowRatio;
					pivotRowIndex = j + 1;
				}
				positiveCount++;
			}
		}

		if(positiveCount==0 || pivotRowIndex==-1){
			print("Row selection failed; Solution Unbounded !!! ");
			break;
		}

		//pivotRowIndex = pivotRowRatios.indexOf(min(pivotRowRatios))+1;
		
		print("Selected Row "+pivotRowIndex);

		// pivotting
		simplexMatrix = pivotElement(simplexMatrix,pivotRowIndex,pivotColumnIndex);
		print(simplexMatrix._data);

		// update basis pivots
		for(var j = 0; j<basisPivots.length; j++){
			if(basisPivots[j][0]==pivotRowIndex){
				basisPivots.splice(j,1,[pivotRowIndex,pivotColumnIndex]);
				break;
			}
		}
		



		if(checkOptimality(simplexMatrix)){
			print("Optimality Reached");
			
			print(basisPivots);
			minCostFlowSolution = getTravelDetails(simplexMatrix,basisPivots);
			print(minCostFlowSolution);
			return getTravelDetails(simplexMatrix,basisPivots);//
			break;
		}else{
			print("Do simplex");
		};
	}

	/*

	
	var A = simplexMatrix.subset(math.index(math.range(1,numberOfNodes),math.range(1,numberOfEdges+1)));
	var b = simplexMatrix.subset(math.index(math.range(1,numberOfNodes),[0]));
	var cB =  simplexMatrix.subset(math.index(0,pickedColumnIndexes));

	*/

}

function drawMinCostFlowSolution(){
	stroke(0);
	if(minCostFlowSolution!=-1){
		for(var i = 0; i<minCostFlowSolution.length; i++){
			line(minCostFlowSolution[i][5].x,minCostFlowSolution[i][5].y,minCostFlowSolution[i][6].x,minCostFlowSolution[i][6].y);
		}
		//line(10,10,200,200);
	}
}

function getTravelDetails(simplexMatrix,basisPivots){// solved tabuleu is inputted

	
	var reducedVariableIndexes = [];
	var reducedVariables = [];
	
	var flowValues = simplexMatrix.subset(math.index(math.range(1,numberOfNodes),[0]))._data;
	for(var i=0; i<basisPivots.length; i++){
		if(flowValues[basisPivots[i][0]-1]!=0){ // 

			reducedVariableIndexes.push(basisPivots[i][1]-1);
			var temp = minCostFlowVariables[basisPivots[i][1]-1];
			temp.push(flowValues[basisPivots[i][0]-1][0]);
			reducedVariables.push(temp);
			
		}
	}

	return reducedVariables;
	//return count;

}

function checkFeasibility(simplexMatrix){
	var b = simplexMatrix.subset(math.index(math.range(1,numberOfNodes),[0]));
	for(var i=1; i<b.length; i++){
		if (b[i]<0){
			return false;
			break;
		}
	}
	return true;

}

function checkOptimality(simplexMatrix){
	var reducedCosts = simplexMatrix._data[0];
	for(var i=1; i<reducedCosts.length; i++){
		if (reducedCosts[i]<0){
			return false;
			break;
		}
	}
	return true;//all are +ve
}

function pivotElement(matrixIn,rowIndex,colIndex){
	var cellValue = matrixIn._data[rowIndex][colIndex];
	
	matrixIn._data[rowIndex] = math.multiply(1/cellValue,matrixIn._data[rowIndex]);//normalize 
	var row = matrixIn._data[rowIndex];

	for(rowScan = 0; rowScan<matrixIn._data.length; rowScan++){
		if(rowScan!=rowIndex){
			cellValue = matrixIn._data[rowScan][colIndex];
			matrixIn._data[rowScan] = math.add(matrixIn._data[rowScan],math.multiply(-1*cellValue,row));
		}
	}

	return matrixIn;

}

function interpretSolution(minCostFlowSolutionTemp,agentID){
//get the travel points for agent i
	var i = agentID;
	var travelPoints = [];
	var travelPointDetails = [];
	var breakIt = false;
	var tempEndPoint;
	
	for(var type = 1; type<5; type ++){//different edge types
		for(var j = 0; j < minCostFlowSolutionTemp.length; j++){// checking through solutions
			if(type==1 && minCostFlowSolutionTemp[j][1]==1 &&minCostFlowSolutionTemp[j][2]==i){// directly jump to destination shadow
				travelPoints.push(minCostFlowSolutionTemp[j][6]);
				travelPointDetails.push("A"+i+"T1A"+i+"S"+minCostFlowSolutionTemp[j][3]+";");
				minCostFlowSolutionTemp.splice(j,1);//entry removed
				breakIt = true;
				break;
			} 
			else if(type==2 && minCostFlowSolutionTemp[j][1]==2 && minCostFlowSolutionTemp[j][2]==i){
				tempEndPoint = [i, minCostFlowSolutionTemp[j][3]];
				travelPoints.push(minCostFlowSolutionTemp[j][6]);//added the first way point of the i-th agent
				travelPointDetails.push("A"+i+"T2A"+i+"W"+minCostFlowSolutionTemp[j][3]+";");
				minCostFlowSolutionTemp.splice(j,1);//entry removed
				break;//check for type 3(wp-wp) or type 4 (wp-sh) next
			}else if(type==3 && minCostFlowSolutionTemp[j][1]==3){
				if(isEqualP2(minCostFlowSolutionTemp[j][5],travelPoints[travelPoints.length-1]) && tempEndPoint[1]==minCostFlowSolutionTemp[j][2] && tempEndPoint[0]==i && minCostFlowSolutionTemp[j][7]>0){//next hop
					tempEndPoint = [i, minCostFlowSolutionTemp[j][3]];
					travelPoints.push(minCostFlowSolutionTemp[j][6]);
					travelPointDetails.push("A"+i+"T3W"+minCostFlowSolutionTemp[j][2]+"W"+minCostFlowSolutionTemp[j][3]+";");
					minCostFlowSolutionTemp[j][7]--; //entry removed
					type--; // do search again type 3
					//break;
					//check for type 4 from this pint if so no need to search for type 3 again;
					for(var k = 0; k<minCostFlowSolutionTemp.length; k++){
						if(minCostFlowSolutionTemp[k][1]==4){
							if(isEqualP2(minCostFlowSolutionTemp[k][5],travelPoints[travelPoints.length-1]) && tempEndPoint[1]==minCostFlowSolutionTemp[k][2] && tempEndPoint[0]==i){
								tempEndPoint = [i, minCostFlowSolutionTemp[k][3]];
								travelPoints.push(minCostFlowSolutionTemp[k][6]);
								travelPointDetails.push("A"+i+"T4W"+minCostFlowSolutionTemp[k][2]+"S"+minCostFlowSolutionTemp[k][3]+"*;");
								minCostFlowSolutionTemp.splice(k,1); //entry removed
								breakIt = true;
								break;
							}

						}
					} 
				}
			}else if(type==4 && minCostFlowSolutionTemp[j][1]==4){
				if(isEqualP2(minCostFlowSolutionTemp[j][5],travelPoints[travelPoints.length-1]) && tempEndPoint[1]==minCostFlowSolutionTemp[j][2] && tempEndPoint[0]==i){//next hop
					tempEndPoint = [i, minCostFlowSolutionTemp[j][3]];
					travelPoints.push(minCostFlowSolutionTemp[j][6]);
					travelPointDetails.push("A"+i+"T4W"+minCostFlowSolutionTemp[j][2]+"S"+minCostFlowSolutionTemp[j][3]+";");
					minCostFlowSolutionTemp.splice(j,1); //entry removed
					breakIt = true;
					break;

				}
				//tempEndPoint = minCostFlowSolution[j][6];
				//print("end added")
			}

		}

		if(breakIt){//particle and shadow connected
			break;
		}
	}

	//print("Travel Points")
	print(travelPointDetails);
	return travelPoints;

}

function isEqualArrays(array1,array2){// compare two number arrays

	if(array1.length!=array2.length){
		return false;
	}else{
		count = 0; 
		for (var i = 0; i < array1.length; i++) {
			if (array1[i]==array2[i]){
				count++;
			}
		}
		if (count==array1.length){
			return true;
		}else{
			return false;
		}
			
	}

}



///////////////////////////////////////////////////////////////// Particle Swarm Optimization Codes

function solvePSO(){


	consolePrint("Start Solving Coverage Control Problem Using Particle Swarm Optimization");

	var numberOfParticlesInPSO = Number(document.getElementById("numOfParticlesPSO").value);
	
	var numberOfAgents = particleShadows.length;

	var bestOfWholeSwarmParticleValues = 0; // best coverage level so far (g)
	var bestOfWholeSwarmParticleLocations = []; // best coverage level so far (g) - particle (array of agents)



	print(numberOfParticlesInPSO);

	// generate 50 candidate solutions
	var cordinateX = 0;
	var cordinateY = 0;
	var samplePoint = new Point2(cordinateX,cordinateY);
	
	var particlesOfPSO = []; // array of arrays (array of particles (particle = array of candidate agents))
	var velocitiesOfparticlesOfPSO = []; // array of arrays (array of particle Velocities(particle Velocity = array of agent velocities))
	
	
	var bestOfEachParticleValue = []; // Store the each particle's best coverage value found so far
	var bestOfEachParticleLocation = []; // store each particle's best Location found so far
	var velocityOfEachParticle = [];

	var particleOfPSO = []; // sample particle
	var velocityOfparticleOfPSO = [];// Point2 with each agent velocity

	consolePrint("Starting Initial Particle Generation for Particle Swarm Optimization");

	for(var i = 0; i<numberOfParticlesInPSO; i++){

		particleOfPSO = [];
		velocityOfparticleOfPSO = [];
		
		for(var j = 0; j<numberOfAgents; j++){
			
			cordinateX = Math.round(width*Math.random());
			cordinateY = Math.round(height*Math.random());
			samplePoint = new Point2(cordinateX,cordinateY)
			
			if(getEventDensity(samplePoint)>0){//valid sample point
				particleOfPSO[j] = new Particle(cordinateX,cordinateY);
				velocityOfparticleOfPSO[j] = new Point2(Math.round(2*width*Math.random()-width)/10,Math.round(2*height*Math.random()-height)/10);// initial velocities of an agent of an particle
			}else{
				j--; // try kagain
			}
		}

		if (particleOfPSO.length==numberOfAgents){
			particlesOfPSO.push(particleOfPSO); //new particle added
			velocitiesOfparticlesOfPSO.push(velocityOfparticleOfPSO);
			
			// best for the particle would be itself so far
			var particleOfPSOCoverageLevel = getCoverageLevelOfPSOParticle(particleOfPSO);
			bestOfEachParticleValue.push(particleOfPSOCoverageLevel); 
			bestOfEachParticleLocation.push(particleOfPSO);

			//updating swarms best
			if(particleOfPSOCoverageLevel>bestOfWholeSwarmParticleValues){// improvement of g
				bestOfWholeSwarmParticleValues = particleOfPSOCoverageLevel;
				bestOfWholeSwarmParticleLocations = particleOfPSO;
				solutionOfPSOTrace.push(bestOfWholeSwarmParticleLocations);
				solutionOfPSO = particleOfPSO; // this will be automatically plotted in blck color
			}

		}else{
			print("Generation Error");
		}

	}

	/*print(particlesOfPSO);
	print(velocitiesOfparticlesOfPSO);
	print(bestOfEachParticleValue);
	print(bestOfEachParticleLocation);
	print(bestOfWholeSwarmParticleValues);
	print(bestOfWholeSwarmParticleLocations);*/
	
	
	consolePrint("Finished Initial Particle Generation for Particle Swarm Optimization; So far optimal: "+Math.round(bestOfWholeSwarmParticleValues)+";"+Math.round(getCoverageLevelOfPSOParticle(solutionOfPSO)));

	// end generation of candidate solutions

	//start updating swarm particles till the
	consolePrint("Initiating Particle Swarm Optimization Update Iterations");

	var PSOParameterPhiP = Number(document.getElementById("PSOPhiP").value);
	var PSOParameterPhiG = Number(document.getElementById("PSOPhiG").value);
	var PSOParameterW = Number(document.getElementById("PSOW").value);
	var numberOfIterations = Number(document.getElementById("PSONit").value);


	////var PSOParameterPhiP = -0.1;
	//// PSOParameterPhiG = -0.5;
	////var PSOParameterW = 0.05;
	////var numberOfIterations = 100; // or can use another termination criterion based on convergence

	var rP; 
	var rG;
	
	
	
	//var particleOfPSOCoverageLevel =0;

	for(var k = 0; k < numberOfIterations; k++){
		for(var i = 0; i < numberOfParticlesInPSO; i++){
			for(var j = 0; j < numberOfAgents; j++){
				// velocityX and positionX update
				rP = Math.random();
				rG = Math.random();
				velocitiesOfparticlesOfPSO[i][j].x = PSOParameterW*velocitiesOfparticlesOfPSO[i][j].x + PSOParameterPhiP*rP*(bestOfEachParticleLocation[i][j].position.x - particlesOfPSO[i][j].position.x) + PSOParameterPhiG*rG*(bestOfWholeSwarmParticleLocations[j].position.x - particlesOfPSO[i][j].position.x);
				

				// velocityY update
				rP = Math.random();
				rG = Math.random();
				velocitiesOfparticlesOfPSO[i][j].y = PSOParameterW*velocitiesOfparticlesOfPSO[i][j].y + PSOParameterPhiP*rP*(bestOfEachParticleLocation[i][j].position.y - particlesOfPSO[i][j].position.y) + PSOParameterPhiG*rG*(bestOfWholeSwarmParticleLocations[j].position.y - particlesOfPSO[i][j].position.y);
				

				// Position updates
				var nextPosition = new Point2(particlesOfPSO[i][j].position.x + velocitiesOfparticlesOfPSO[i][j].x,particlesOfPSO[i][j].position.y + velocitiesOfparticlesOfPSO[i][j].y);

				nextPosition = particlesOfPSO[i][j].avoidObstacles(nextPosition);// projected next position avoiding obstacles

				if(getEventDensity(nextPosition)>0){
					particlesOfPSO[i][j].position = nextPosition;
				}
			}

			// update particle best
			var particleOfPSOCoverageLevel = getCoverageLevelOfPSOParticle(particlesOfPSO[i]);
			if(particleOfPSOCoverageLevel > bestOfEachParticleValue[i]){
				bestOfEachParticleValue[i] = particleOfPSOCoverageLevel; 
				bestOfEachParticleLocation[i] = particlesOfPSO[i];

				//updating swarms best
				if(particleOfPSOCoverageLevel > bestOfWholeSwarmParticleValues){// improvement of g
					bestOfWholeSwarmParticleValues = particleOfPSOCoverageLevel;
					bestOfWholeSwarmParticleLocations = bestOfEachParticleLocation[i];
					solutionOfPSOTrace.push(bestOfWholeSwarmParticleLocations);
					solutionOfPSO = bestOfEachParticleLocation[i];
					consolePrint("Update:"+Math.round(bestOfWholeSwarmParticleValues)+";"+Math.round(getCoverageLevelOfPSOParticle(solutionOfPSO))+";");
					//print("bfbdf");
					//print(solutionOfPSO);
				} 
			}



		}
	}
	//print(solutionOfPSO);
	//solutionOfPSO = bestOfWholeSwarmParticleLocations; // this will be automatically plotted in blck color
	consolePrint("Finished Particle Swarm Optimization Iterations; Optimal Coverage: "+Math.round(bestOfWholeSwarmParticleValues)+";"+Math.round(getCoverageLevelOfPSOParticle(solutionOfPSO)));
	//print(solutionOfPSOTrace);
	assignSolutionTracesOfPSO(solutionOfPSOTrace)

	// end updating swarm particles

}



function getCoverageLevelOfPSOParticle(agentArray){

	var globalObjectiveValue = 0;
    var stepSize = 10; 
    var halfStepSize = stepSize/2;
    var areaFactor = sq(stepSize);
    

    for (var x = halfStepSize; x <= width - halfStepSize; x+=stepSize){
        for(var y = halfStepSize; y <= height - halfStepSize; y+=stepSize){
            
            var interestedPoint = new Point2(x,y);
            var eventDensity = getEventDensity(interestedPoint);
            
            if(eventDensity>0){
                //print(detectionProbabilityGlobal(interestedPoint));
                globalObjectiveValue = globalObjectiveValue + detectionProbabilityGlobalForPSO(interestedPoint,agentArray)*eventDensity*areaFactor;
            }
            
        }
    }

    return globalObjectiveValue;
}


function detectionProbabilityGlobalForPSO(interestedPoint,agentArray){
    var jointMissProbability = 1;
    
    if(agentArray.length==0){//no particleShadows - nothing to detect
        return 0;
    }
    else{
        for (var i = 0; i < agentArray.length; i++){
            //print(particles[i].sensingModelFunction(interestedPoint));
            jointMissProbability = jointMissProbability*(1-agentArray[i].sensingModelFunction(interestedPoint));
        }
    }
    return (1-jointMissProbability);
}


function showParticleOfPSO(agentArray){//agent array = particleOfPSO

	var pointArrayNew = [];
	for (var i = 0; i<agentArray.length; i++) {
		pointArrayNew.push(agentArray[i].position);
		//print(solutionOfPSO[i])
		//print(agentArray[i]);
	}

	//stroke(128,0,0);
	for(var i=0 ; i<pointArrayNew.length; i++){
		fill(agentArray[i].ID);
		stroke(agentArray[i].ID);
	
		ellipse(pointArrayNew[i].x, pointArrayNew[i].y, 10, 10);
		
	}

	//print("here");
	//print(pointArrayNew);
	////printPointArrayP2(pointArrayNew,solutionOfPSO[j].ID,10);


}

function assignSolutionTracesOfPSO(solutionTrace){
	for(var i = 0; i<solutionTrace.length; i++){
		for(var j=0; j<solutionTrace[i].length;j++){
			solutionOfPSO[j].trajectory.push(solutionTrace[i][j].position);
		}
	}
	for(var j=0; j<solutionOfPSO.length;j++){
		solutionOfPSO[j].ID = getRandomColor();
	}
}


function getRandomColor() {
  var letters = '0123456789ABCDEF';
  var color = '#';
  for (var i = 0; i < 6; i++) {
    color += letters[Math.floor(Math.random() * 16)];
  }
  return color;
}


function playPSO(){
	pointsToBePrinted = [];
	simulatePSOInterval = setInterval(simulatePSO,1000);
}

function stopPSO(){
	pointsToBePrinted = [];
	clearInterval(simulatePSOInterval);
}

function simulatePSO(){
	
	pointsToBePrinted = [];
	for (var i = 0; i < solutionOfPSO.length; i++) {
        printPointArrayP2(solutionOfPSO[i].trajectory[simulatePSOIntervalStep],solutionOfPSO[i].ID,3);
    	print(solutionOfPSO[i].trajectory[simulatePSOIntervalStep].x);
    	pointsToBePrinted.push(solutionOfPSO[i].trajectory[simulatePSOIntervalStep]);
    }

    simulatePSOIntervalStep++;
    if(solutionOfPSO[0].trajectory.length==simulatePSOIntervalStep){
    	simulatePSOIntervalStep = 0;
    }

}

function printPointPSO(position,colorValue,radius){
	fill(colorValue);
    stroke(0);
	ellipse(position.x, position.y, 20, 20);
}





////////////////////////// Submodularity Related stuff

function addCustomCandidatesBtnFcn(){
	for(var i = 0; i<particleShadows.length; i++){

		submodularityCandidates.push(particleShadows[i].position);
		
	}
}

function generateCandidatesBtnFcn(){

	consolePrint("Solving Coverage Control Problem Using Greedy Algorithm: Started");

	document.getElementById("displayCoverageDensity").checked = false;

	var numberOfCandidates = Number(document.getElementById("numOfCandidatesSubmodularity").value);	

	// global variable reset
	submodularityCandidates = [];
	submodularityMode = 1;

	consolePrint("Initial Candidate Points Generation: Started. Please wait....");


	//  Random point generation
	// for(var i = 0; i<numberOfCandidates; i++){

	// 	cordinateX = Math.round(width*Math.random());
	// 	cordinateY = Math.round(height*Math.random());
	// 	samplePoint = new Point2(cordinateX,cordinateY)
		
	// 	if(getEventDensity(samplePoint)>0){//valid sample point
	// 		submodularityCandidates[i] = samplePoint;
	// 	}else{
	// 		i--; // try kagain
	// 	}
		
	// }
	// end  Random point generation
	
	
	// coordinated point generation
	var sqn = Math.sqrt(numberOfCandidates); // agents per line
	var delta = width/sqn; //or height/sqn
	var samplePoint; 
	var missedPointCount = 0; 
	var pointCount = 0;

	for(var y=delta/2; y<=(height-(delta/2)); y=y+delta){
		for(var x=delta/2; x<=(width-(delta/2)); x=x+delta){
			samplePoint = new Point2(x,y);
			if(getEventDensity(samplePoint)>0){//valid sample point
				submodularityCandidates[pointCount] = samplePoint;
				pointCount++;
			}		
		}
	}

	var remainingPoints = numberOfCandidates - pointCount;
	print(remainingPoints);
	for(var i = 0; i<remainingPoints; i++){

		var x = Math.round(width*Math.random());
		var y = Math.round(height*Math.random());
		samplePoint = new Point2(x,y);
		
		if(getEventDensity(samplePoint)>0){//valid sample point
			submodularityCandidates[pointCount] = samplePoint;
			pointCount++;
		}else{
			i--; // try kagain
		}
		
	}
	// end coordinated point generation

	consolePrint("Initial Candidate Points Generation: Finished.");

	//if(sqn<15){calculateApproxFactorsBtnFcn()};
}


function readjustCandidatesStart(){
	
	
	if(submodularityMode==2){// quick kill
		for(var i = 0; i < particleShadows.length; i++){
            submodularityCandidates[i] = particleShadows[i].position; 
        }
        printPointArrayP2(submodularityCandidates,color(100,0,100),2);
        readjustCandidatesEnd();
	}else{
		// end quick kill

		consolePrint("Updating Initial Candidate Points: Started. Please wait...");
		// save the existing particle shadows
		savedParticles = particleShadows;
		

		// remove all the existing agents
		particleShadows = [];particles = [];

		// adjust sensing range
		savedParameters[0] = Number(document.getElementById("sensingRange").value);
		var sqn = Math.sqrt(submodularityCandidates.length); // agents per line
		senRange = width/(2*sqn);
		if(sqn>20){senRange = width/(10*sqn);}
		document.getElementById("sensingRange").value = senRange;
	    document.getElementById("sensingRangeDisplay").innerHTML = senRange;


	    // adjust descritizationLevel
	    savedParameters[1] = Number(document.getElementById("descretizationLevelVal").value);
	    descretizationLevel = Math.ceil(senRange/5);
	    if(sqn>20){descretizationLevel = senRange/20}
	    document.getElementById("descretizationLevelVal").value = descretizationLevel;
	    document.getElementById("descretizationLevelDisplay").innerHTML = descretizationLevel;


		// load each candidate  as a particleShadow
		for(var i = 0; i < submodularityCandidates.length; i++){
			var newParticle = new Particle(submodularityCandidates[i].x, submodularityCandidates[i].y);
			newParticle.senRange = senRange;
			newParticle.sensingDecayFactor = sensingDecayFactor; 
			particleShadows.push(newParticle);
		}

		// run coverage control iterations (1000 iterations or so)
		savedParameters[2] = 200;
		// relaod prevously saved particle shadows
		submodularityMode = 2; 
	}

}

function readjustCandidatesEnd(){// finish the readjustment
	// relaod prevously saved particle shadows
	particleShadows = savedParticles;

	// readjust sensing range setting
	document.getElementById("sensingRange").value = savedParameters[0];
	senRange = savedParameters[0];

	// readjust candidate set
	document.getElementById("descretizationLevelVal").value = savedParameters[1];
	descretizationLevel = savedParameters[1];

	document.getElementById("displayCoverageDensity").checked = true;

	submodularityMode = 1; 
	consolePrint("Updating Initial Candidate Points: Finished.");


	// adjust sensing ranges/decay factors of each individual agents

}



// Calculate approximation factors
function calculateApproxFactorsBtnFcn(){
	
	//// calculating the total curvature
	// save the existing particle shadows
	savedParticles = particleShadows;
	var N = savedParticles.length;
	var Th_N = 1-Math.pow(1-(1/N),N);// theoretical approx factor

	// remove all the existing agents
	removeAllAgents();
	////particleShadows = [];particles = [];
	// load each candidate  as a particleShadow
	for(var i = 0; i < submodularityCandidates.length; i++){
		particleShadows.push(new Particle(submodularityCandidates[i].x, submodularityCandidates[i].y));
	}
	
	var valueArray = [];
	var H1 = 0; 
	var H2 = 0;
	for(var i = 0; i<particleShadows.length; i++){
		H1 = particleShadows[i].localObjectiveFunction();
		H2 = particleShadows[i].localObjectiveFunctionWRTNeighbors([]);
		if(isNaN(H1/H2)){
			valueArray.push(0);
		}else{
			valueArray.push(1-(H1/H2));	
		}
		////print(i);
	} 
	//print(valueArray);
	var c = valueArray.reduce(function(a, b){return Math.max(a, b);});
	var T_cN = (1/c)*(1-Math.pow(1-(c/N),N));
	

	//// elemenntal curvature
	// remove all the existing agents
	////particleShadows = [];particles = [];
	removeAllAgents();
	var p_j;
	var p_k;
	var exitLoops = false;
	var valueArray = [];
	var maxSoFar = 0;
	var maxTemp;
	for(var k = 0; k < submodularityCandidates.length; k++){
		
		particleShadows.push(new Particle(submodularityCandidates[k].x, submodularityCandidates[k].y));
		for(var j = 0; j < submodularityCandidates.length; j++){
			particleShadows.push(new Particle(submodularityCandidates[j].x, submodularityCandidates[j].y));
			
			valueArray = [];
			for(var ind = 0; ind < submodularityCandidates.length; ind++){
				p_k = particleShadows[0].sensingModelFunction(submodularityCandidates[ind]);
				if(p_k>0){
					p_j = particleShadows[1].sensingModelFunction(submodularityCandidates[ind]);
					if(p_j>0){
						valueArray.push(1-p_j);
					}
				}
			}
			if(valueArray.length==0){
				maxSoFar = 1;
				removeAgent();
				exitLoops = true;
				break;
			}else{
				maxTemp = valueArray.reduce(function(a, b){return Math.max(a, b);});
				if(maxTemp>maxSoFar){
					maxSoFar = maxTemp;
				}
			}
			removeAgent();
		}
		if(exitLoops){
			maxSoFar = 1;
			removeAgent();
			/////print(maxSoFar);
			break;
		}
		removeAgent();
		

	}
	
	var alpha = maxSoFar;
	var E_cN;
	if(alpha==1){
		E_cN = 1-Math.pow(1-(1/N),N);
	}else{// when alpha<1
		var alphaSum = 0;
		for(i=1;i<N;i++){
			alphaSum = alphaSum + Math.pow(alpha,i);
		}
		E_cN = 1-Math.pow(alphaSum/(1+alphaSum),N);
	}

	// end - elemental curvature based approx factor calculation




	//// partial curvature based approx factor
	// remove all the existing agents
	removeAllAgents();
	////particleShadows = [];particles = [];
	var chosenCandidateIndexes = [];
	// addding first agent j
	var costArray = [];
	var classArray = [];
	var classification = classify(savedParticles);// grouping agents
	
	for(var j = 0; j<submodularityCandidates.length; j++){// adding scanning candidates
		
		var subCostArray = [];
		for(var k = 0; k < classification.length; k++){
			if(classification[k].length>0){// has remaining agents
				//add first remaining agent of the class i 
				addClassifiedAgentToPoint(classification[k][0],submodularityCandidates[j].x,submodularityCandidates[j].y);
				subCostArray[k] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;	
				removeAgent();
			}else{
				subCostArray[k] = 0; // to discourage  
			}
		}
		var chosenClassIndex = subCostArray.indexOf(subCostArray.reduce(function(a, b){return Math.max(a, b);}));
		addClassifiedAgentToPoint(classification[chosenClassIndex][0], submodularityCandidates[j].x,submodularityCandidates[j].y);
		classArray[j] = chosenClassIndex;

		costArray[j] = subCostArray[chosenClassIndex];
		removeAgent(); // remove last element

	}
	var chosenCandidateIndex = costArray.indexOf(costArray.reduce(function(a, b){return Math.max(a, b);}));
	var chosenCandidateClassIndex = classArray[chosenCandidateIndex];
	var solutionPoint = submodularityCandidates[chosenCandidateIndex];

	addClassifiedAgentToPointFinal(classification[chosenCandidateClassIndex][0],solutionPoint.x,solutionPoint.y);
	chosenCandidateIndexes.push(chosenCandidateIndex);
	classification[chosenCandidateClassIndex].shift();
	// end adding first agent


	// need to add the rest of agents (i.e. N-1 agents so that)
	for(var i = 1; i<N; i++){
		var costArray = [];
		var classArray = []; // classification is used from memmory
		for(var j = 0; j<submodularityCandidates.length; j++){// adding scanning candidates
			if(chosenCandidateIndexes.includes(j)){// not choses previously
				costArray[j] = 1000000; // will disencourage chosing this point
			}else{

				var subCostArray = [];
				for(var k = 0; k < classification.length; k++){
					if(classification[k].length>0){// has remaining agents
						//add first remaining agent of the class i 
						addClassifiedAgentToPoint(classification[k][0],submodularityCandidates[j].x,submodularityCandidates[j].y);
						subCostArray[k] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;	
						removeAgent();
					}else{
						subCostArray[k] = 1000000; // to discourage  
					}
				}
				var chosenClassIndex = subCostArray.indexOf(subCostArray.reduce(function(a, b){return Math.min(a, b);})); // need to add the worst agent to the considered point
				addClassifiedAgentToPoint(classification[chosenClassIndex][0], submodularityCandidates[j].x,submodularityCandidates[j].y);
				classArray[j] = chosenClassIndex;

				costArray[j] = subCostArray[chosenClassIndex];
				removeAgent(); // remove last element

			} 
		}
		var chosenCandidateIndex = costArray.indexOf(costArray.reduce(function(a, b){return Math.min(a, b);}));
		var chosenCandidateClassIndex = classArray[chosenCandidateIndex];
		var solutionPoint = submodularityCandidates[chosenCandidateIndex];
		
		addClassifiedAgentToPointFinal(classification[chosenCandidateClassIndex][0],solutionPoint.x,solutionPoint.y);
		chosenCandidateIndexes.push(chosenCandidateIndex);
		classification[chosenCandidateClassIndex].shift();
	}
	var b_H = 1 - particleShadows[0].localObjectiveFunction()/particleShadows[0].localObjectiveFunctionWRTNeighbors([]);
	var P_cN = (1/b_H)*(1-Math.pow(1-(b_H/N),N));
	// end partial curvature calculation
	print("Neighbors:"+particleShadows[0].getNeighbors());
	print("P(s_i|Neighbors):"+particleShadows[0].localObjectiveFunction());
	print("P(s_i|empty):"+particleShadows[0].localObjectiveFunctionWRTNeighbors([]));
	print("b_H"+b_H);
	print("P_cN"+P_cN);



	//// greedy curvature based:
	removeAllAgents();
	////particleShadows = [];particles = [];

	// chosen candidate indexes
	savedParameters[0] = [];
	savedParameters[1] = 0; // length of current particle shadows
	savedParameters[2] = 0; // dummy for now
	savedParameters[3] = []; //[max_i(1-H/H); i = iteration number] - for greedy curvature calculation
	var classification = classify(savedParticles);// grouping agents
	savedParameters[5] = classification; 

	var numberOfCandidates = submodularityCandidates.length;
	for(var i = 0; i<N; i++){
		var chosenCandidateIndexes = savedParameters[0];
		var classArray = [];
		var costArray = [];
		var greedyCurvatureArray = []; //1-H(s_j|S^i)/H(s_j|\Phi)
		var H_jValueWRTPhi = 0;
		for(var j = 0; j<numberOfCandidates; j++){// scanning candidates
			
			if(chosenCandidateIndexes.includes(j)){// not choses previously
				costArray[j] = 0; // will disencourage chosing this point
				greedyCurvatureArray[j] = 0;
			}else{
				// lets try adding agents of different classes
				var subCostArray = [];
				for(var k = 0; k < classification.length; k++){
					if(classification[k].length>0){// has remaining agents
						//add first remaining agent of the class i 
						addClassifiedAgentToPoint(classification[k][0],submodularityCandidates[j].x,submodularityCandidates[j].y);
						subCostArray[k] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;	
						removeAgent();
					}else{
						subCostArray[k] = 0; // to discourage  
					}
				}
				var chosenClassIndex = subCostArray.indexOf(subCostArray.reduce(function(a, b){return Math.max(a, b);}));
				print('Debug '+chosenClassIndex);
				print(submodularityCandidates[j]);
				addClassifiedAgentToPoint(classification[chosenClassIndex][0], submodularityCandidates[j].x,submodularityCandidates[j].y);
				classArray[j] = chosenClassIndex;

				costArray[j] = subCostArray[chosenClassIndex];
				H_jValueWRTPhi = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunctionWRTNeighbors([])*1000)/1000;
				greedyCurvatureArray[j] = 1-(costArray[j]/H_jValueWRTPhi);
				if(isNaN(greedyCurvatureArray[j])){greedyCurvatureArray[j]=0;}
				removeAgent(); // remove last element
			} 

		}
		var chosenCandidateIndex = costArray.indexOf(costArray.reduce(function(a, b){return Math.max(a, b);}));
		var chosenCandidateClassIndex = classArray[chosenCandidateIndex];
		var solutionPoint = submodularityCandidates[chosenCandidateIndex];


		addClassifiedAgentToPointFinal(classification[chosenCandidateClassIndex][0],solutionPoint.x,solutionPoint.y);
		chosenCandidateIndexes.push(chosenCandidateIndex);
		savedParameters[0] = chosenCandidateIndexes;
		savedParameters[1] = savedParameters[1]+1;
		
		// print("Iteration "+i);
		// print(submodularityCandidates[greedyCurvatureArray.indexOf(greedyCurvatureArray.reduce(function(a, b){return Math.max(a, b);}))]);
		// print(greedyCurvatureArray.reduce(function(a, b){return Math.max(a, b);}));
		

		savedParameters[3].push(greedyCurvatureArray.reduce(function(a, b){return Math.max(a, b);}));

		classification[chosenCandidateClassIndex].shift();
		//print(classification)
		savedParameters[5] = classification;
	}

	// print("Global best");
	// print(savedParameters[3].indexOf(savedParameters[3].reduce(function(a, b){return Math.max(a, b);})))
	
	var alpha_g = savedParameters[3].reduce(function(a, b){return Math.max(a, b);});
	var G_cN = (1-alpha_g)*(1-(1/N));
	// end greedy curvature calculation





	Th_N = Math.round(10000000*Th_N)/10000000;
	T_cN = Math.round(10000000*T_cN)/10000000;
	E_cN = Math.round(10000000*E_cN)/10000000;
	P_cN = Math.round(10000000*P_cN)/10000000;
	G_cN = Math.round(10000000*G_cN)/10000000;

	if(savedParameters[5].length>1){
		T_cN = "N/A";// undefined!
		E_cN = "N/A";
	}
	
	consolePrint("Approximation factors calculated for this mission space configuration (with "+savedParameters[5].length+" agent classes) are as follows:");
	consolePrint("Theoretical:- "+Th_N+", Total Cur.:- "+T_cN+", Elemental Cur.:- "+E_cN+", Partial Cur.:- "+P_cN+", Greedy Cur.:- "+G_cN+".");
	


	// relaod prevously saved particle shadows
	particleShadows = savedParticles;


}
// end appproximation factors



// Centralized general greedy Method
function solveCentralizedGreedyBtnFcn(){
	
	var classification = classify(particleShadows);
	consolePrint("Solving Using Centralized General Greedy Algorithm (with "+classification.length+" agent classes): Started. Please wait...");
	
	var classString = "";
	for(var i = 0; i<classification.length; i++){
		var agentList = "[";
		for(var j = 0; j< classification[i].length; j++){
			if(j<classification[i].length-1){
				agentList = agentList + (classification[i][j]+1)+",";
			}else{
				agentList = agentList + (classification[i][j]+1);
			}
		}
		agentList = agentList + "]"
		classString = classString + "Class "+(i+1)+": agents= "+agentList+", with  R = "+particleShadows[classification[i][0]].senRange+", and d = "+particleShadows[classification[i][0]].sensingDecayFactor+";\n";	
	}
	consolePrint(classString);
	
	// save the existing particle shadows
	savedParticles = particleShadows;

	// remove all the agents
	removeAllAgents();  

	// empty particleShadows = [];particles = [];
	// end remove all agents


	// chosen candidate indexes
	savedParameters[0] = [];
	savedParameters[1] = 0; // length of current particle shadows
	savedParameters[2] = 0; // number of computations did (to check O(nN))
	savedParameters[3] = []; //[max_i(1-H/H); i = iteration number] - for greedy curvature calculation
	savedParameters[4] = []; // used for continuous greedy method - unused here

	//separate agents into classes depending on their sensing capabilities (range and decay)
	savedParameters[5] = classification; 
	savedParameters[6] = []; // order of the added agents

	// need to ad one agent by one agent,
	// each agent should take the best candidate position ( to maximize H(s) - global ) remaining
	submodularityMode = 3; 
	
	executionTimeGreedy = 0;
	
}

function iterationOfCentralizedGreedy(){
	
	var numberOfCandidates = submodularityCandidates.length;
	var chosenCandidateIndexes = savedParameters[0];
	var classification = savedParameters[5];
	////print(classification)
	var classArray = [];

	var costArray = [];
	var greedyCurvatureArray = []; //1-H(s_j|S^i)/H(s_j|\Phi)
	var H_jValueWRTPhi = 0;
	var computations = 0;
	for(var j = 0; j<numberOfCandidates; j++){// scanning candidates
		
		if(chosenCandidateIndexes.includes(j)){// not choses previously
			costArray[j] = 0; // will disencourage chosing this point
			greedyCurvatureArray[j] = 0;
		}else{


			// lets try adding agents of different classes
			var subCostArray = [];
			for(var i = 0; i < classification.length; i++){
				if(classification[i].length>0){// has remaining agents
					//add first remaining agent of the class i 
					addClassifiedAgentToPoint(classification[i][0],submodularityCandidates[j].x,submodularityCandidates[j].y);
					subCostArray[i] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;	
					removeAgent();
				}else{
					subCostArray[i] = -1; // to discourage  
				}
			}
			var chosenClassIndex = subCostArray.indexOf(subCostArray.reduce(function(a, b){return Math.max(a, b);}));
			////print("chosenClassIndex = "+chosenClassIndex+ "costs: "+subCostArray);
			addClassifiedAgentToPoint(classification[chosenClassIndex][0], submodularityCandidates[j].x,submodularityCandidates[j].y);
			classArray[j] = chosenClassIndex;
			



			// the following line  can be improved by considering only the cost increment of adding an agent 
			//costArray[j] = Math.round(globalObjective()*1000)/1000;
			// improved version: cost increment is only calculated using local info.
			////costArray[j] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;
			costArray[j] = subCostArray[chosenClassIndex]; 
			H_jValueWRTPhi =  Math.round(particleShadows[particleShadows.length-1].localObjectiveFunctionWRTNeighbors([])*1000)/1000;
			greedyCurvatureArray[j] = 1-(costArray[j]/H_jValueWRTPhi);
			if(isNaN(greedyCurvatureArray[j])){greedyCurvatureArray[j]=0;}
			removeAgent(); // remove last element
			computations++;
		} 

	}
	
	////var chosenCandidateIndex = costArray.indexOf(Math.max.apply(Math,costArray));
	// same as follows
	var chosenCandidateIndex = costArray.indexOf(costArray.reduce(function(a, b){return Math.max(a, b);}));
	var chosenCandidateClassIndex = classArray[chosenCandidateIndex];
	var solutionPoint = submodularityCandidates[chosenCandidateIndex];

	//print(chosenCandidateIndex,solutionPoint);

	addClassifiedAgentToPointFinal(classification[chosenCandidateClassIndex][0],solutionPoint.x,solutionPoint.y);
	chosenCandidateIndexes.push(chosenCandidateIndex);
	savedParameters[0] = chosenCandidateIndexes;
	savedParameters[1] = savedParameters[1]+1;
	savedParameters[2] = savedParameters[2]+computations;
	savedParameters[3].push(greedyCurvatureArray.reduce(function(a, b){return Math.max(a, b);}));
	
	classification[chosenCandidateClassIndex].shift();
	print(classification)
	savedParameters[5] = classification;
	savedParameters[6].push(chosenCandidateClassIndex);
}
// end - Centralized general greedy Method




// centralized general greedy method - continuous version
function solveCentralizedGreedyContinuousBtnFcn(){

	consolePrint("Solving Using Centralized - General Greedy Algorithm: Started. Please wait...");
	// save the existing particle shadows
	savedParticles = particleShadows;

	// remove all the agents
	removeAllAgents();
	////particleShadows = [];particles = [];

	// chosen candidate indexes
	savedParameters[0] = [];
	savedParameters[1] = 0; // length of current particle shadows
	savedParameters[2] = 0; // number of computations did (to check O(nN))
	savedParameters[3] = 0; // to find the convergence
	savedParameters[4] = []; //[max_i(1-H/H); i = iteration number] - for greedy curvature calculation
	// need to ad one agent by one agent,
	// each agent should take the best candidate position ( to maximize H(s) - global ) remaining
	submodularityMode = 3.5; 
}
function iterationOfCentralizedGreedyContinuous(){
	
	var numberOfCandidates = submodularityCandidates.length;
	var chosenCandidateIndexes = savedParameters[0];
	
	var costArray = [];
	var greedyCurvatureArray = []; //1-H(s_j|S^i)/H(s_j|\Phi)
	var H_jValueWRTPhi = 0;
	var computations = 0;
	for(var j = 0; j<numberOfCandidates; j++){// scanning candidates
		
		if(chosenCandidateIndexes.includes(j)){// not choses previously
			costArray[j] = 0; // will disencourage chosing this point
			greedyCurvatureArray[j] = 0;
		}else{
			addAgentToPoint(submodularityCandidates[j].x,submodularityCandidates[j].y);
			
			// the following line  can be improved by considering only the cost increment of adding an agent 
			//costArray[j] = Math.round(globalObjective()*1000)/1000;
			// improved version: cost increment is only calculated using local info.
			costArray[j] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;
			H_jValueWRTPhi = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunctionWRTNeighbors([])*1000)/1000;
			greedyCurvatureArray[j] = 1-(costArray[j]/H_jValueWRTPhi);
			if(isNaN(greedyCurvatureArray[j])){greedyCurvatureArray[j]=0;}
			removeAgent(); // remove last element
			computations++;
		} 

	}
	////var chosenCandidateIndex = costArray.indexOf(Math.max.apply(Math,costArray));
	var chosenCandidateIndex = costArray.indexOf(costArray.reduce(function(a, b){return Math.max(a, b);}));
	var solutionPoint = submodularityCandidates[chosenCandidateIndex];

	//print(chosenCandidateIndex,solutionPoint);

	addAgentToPoint(solutionPoint.x,solutionPoint.y);
	chosenCandidateIndexes.push(chosenCandidateIndex);
	savedParameters[0] = chosenCandidateIndexes;
	savedParameters[1] = savedParameters[1]+1;
	savedParameters[2] = savedParameters[2]+computations;
	savedParameters[4].push(greedyCurvatureArray.reduce(function(a, b){return Math.max(a, b);}));
	
}	
// end - Centralized general greedy Method - continuoous mode




// Centralized Stochastic greedy Method 
function solveCentralizedStochasticGreedyBtnFcn(){
	consolePrint("Solving Using Centralized - Stochastic Greedy Algorithm: Started. Please wait...");
	// save the existing particle shadows
	savedParticles = particleShadows;

	// remove all the agents
	particleShadows = [];particles = [];

	// chosen candidate indexes
	savedParameters[0] = [];
	savedParameters[1] = 0; // length of current particle shadows
	savedParameters[2] = 0; // number of computations did (to check O(nN))
	savedParameters[3] = []; // set of reduced candidates (randomly picked at different stages)


	// need to ad one agent by one agent,
	// each agent should take the best candidate position ( to maximize H(s) - global ) remaining
	submodularityMode = 4; 
}

function iterationOfCentralizedStochasticGreedy(){

	var numberOfCandidates = submodularityCandidates.length;
	var N = savedParticles.length; // k or total num of agents we seek to fill
	var chosenCandidateIndexes = savedParameters[0];

	var reducedSetOfIndexes = [];
	
	for(var i=0; i<numberOfCandidates; i++){
		if(chosenCandidateIndexes.includes(i)){
			//pass
		}else{
			reducedSetOfIndexes.push(i);
		}
	}

	var epsilon = Number(document.getElementById("epsilonStochasticGreedy").value);
	var sizeOfSelection = Math.ceil((numberOfCandidates/N)*Math.log(1/epsilon)); //actually depends on the choice of epsilon
	
	if(sizeOfSelection>=reducedSetOfIndexes.length){
		sizeOfSelection = reducedSetOfIndexes.length;
		print("No effect of epsilon on the total number of computations needs to be done!");
	}else if(sizeOfSelection<=0){
		sizeOfSelection = 1;
		print("Errorneous subset size");
	}

	// reducing the set of candidates by uniform random sampling
	var selectedCandidateIndexes = getRandomSubarray(reducedSetOfIndexes, sizeOfSelection);
	
	// display purposes
	var reducedSetOfPoints = [];

	// greedy iteration on the reduced set
	var costArray = [];
	var computations = 0;

	for(var i = 0; i<selectedCandidateIndexes.length; i++){// scanning candidates
		
		var j = selectedCandidateIndexes[i]; //corresponding index of the original array
		reducedSetOfPoints.push(submodularityCandidates[j]);// display purposes

		// not worring aabout 
		addAgentToPoint(submodularityCandidates[j].x,submodularityCandidates[j].y);
		
		// the following line  can be improved by considering only the cost increment of adding an agent 
		// costArray[j] = Math.round(globalObjective()*1000)/1000;
		// improved version: cost increment is only calculated using local info.
		costArray[i] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;
		
		removeAgent(); // remove last element
		computations++;

	}
	var chosenCandidateIndex = costArray.indexOf(Math.max.apply(Math,costArray));
	chosenCandidateIndex = selectedCandidateIndexes[chosenCandidateIndex];
	var solutionPoint = submodularityCandidates[chosenCandidateIndex];

	//print(chosenCandidateIndex,solutionPoint);

	addAgentToPoint(solutionPoint.x,solutionPoint.y);
	chosenCandidateIndexes.push(chosenCandidateIndex);
	savedParameters[0] = chosenCandidateIndexes;
	savedParameters[1] = savedParameters[1]+1;
	savedParameters[2] = savedParameters[2]+computations;
	savedParameters[3] = reducedSetOfPoints;

}

function getRandomSubarray(arr, size) {
	// from: https://stackoverflow.com/questions/11935175/sampling-a-random-subset-from-an-array
    var shuffled = arr.slice(0), i = arr.length, min = i - size, temp, index;
    while (i-- > min) {
        index = Math.floor((i + 1) * Math.random());
        temp = shuffled[index];
        shuffled[index] = shuffled[i];
        shuffled[i] = temp;
    }
    return shuffled.slice(min);
}
// End Centralized Stochastic greedy Method







// Distributed Randomized Greedy method
function solveDistributedRandomizedGreedyBtnFcn(){
	
	consolePrint("Solving Using Distributed - Randomized Greedy Algorithm: Started. Please wait...");
	// save the existing particle shadows
	savedParticles = particleShadows;

	// remove all the agents
	particleShadows = [];particles = [];

	
	savedParameters[0] = []; // chosen candidate indexes (by each machine)
	savedParameters[1] = 0; // length of current particle shadows (agents added so far)
	savedParameters[2] = 0; // number of computations did (to check O(nN))
	savedParameters[3] = []; // set of reduced candidates for each machine (for display purposes)

	// grouping candidate points into m groups
	var numOfMachines = Number(document.getElementById("mRandomizedGreedy").value);//m values
	var machineAssignments = [];
	for(var machine=0;machine<numOfMachines;machine++){
		machineAssignments.push([]);
		savedParameters[0].push([]);
		savedParameters[3].push([]);// display purposes
	}
	for(var i = 0; i<submodularityCandidates.length; i++){
		var randMachine = Math.floor(Math.random() * numOfMachines);
		machineAssignments[randMachine].push(i);
		// for display purposes:
		savedParameters[3][randMachine].push(submodularityCandidates[i]);
		// assign each candidate index to a row (here each row indicates a different machine)
	}

	savedParameters[4] = 0; // current machine number
	savedParameters[5] = machineAssignments;
	savedParameters[6] = []; //objective function value of each machine
	// now we need to calculate greedy algorithm (centralized general version) one by one
	submodularityMode = 5;


	// grouping
}


function iterationOfDistributedRandomizedGreedy(currentMachine){
	
	var reducedIndexes = savedParameters[5][currentMachine];

	// normal greedy
	var chosenCandidateIndexes = savedParameters[0][currentMachine];
	
	var costArray = [];
	var computations = 0;
	for(var i = 0; i<reducedIndexes.length; i++){// scanning candidates
		
		var j = reducedIndexes[i];

		if(chosenCandidateIndexes.includes(j)&&!(reducedIndexes.length<=chosenCandidateIndexes.length)){// not choses previously
			costArray[i] = 0; // will disencourage chosing this point
		}else{
			addAgentToPoint(submodularityCandidates[j].x,submodularityCandidates[j].y);
			
			// the following line  can be improved by considering only the cost increment of adding an agent 
			//costArray[j] = Math.round(globalObjective()*1000)/1000;
			// improved version: cost increment is only calculated using local info.
			costArray[i] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;
			
			removeAgent(); // remove last element
			computations++;
		} 

	}
	var chosenCandidateIndex = costArray.indexOf(Math.max.apply(Math,costArray));
	chosenCandidateIndex = reducedIndexes[chosenCandidateIndex];
	var solutionPoint = submodularityCandidates[chosenCandidateIndex];

	//print(chosenCandidateIndex,solutionPoint);

	addAgentToPoint(solutionPoint.x,solutionPoint.y);
	chosenCandidateIndexes.push(chosenCandidateIndex);
	savedParameters[0][currentMachine] = chosenCandidateIndexes;
	savedParameters[1] = savedParameters[1]+1;
	savedParameters[2] = savedParameters[2]+computations;

}

function centralizedIterationOfTheDistributedRandomizedGreedy(currentMachine){
	// for this iterarion, currentMachine==numOfMachines
	iterationOfDistributedRandomizedGreedy(currentMachine);// nothing else required

}

function sleepFor(miliseconds) {
   var currentTime = new Date().getTime();

   while (currentTime + miliseconds >= new Date().getTime()) {
   }
}
// End - Distributed Randomized Greedy method





// Distributed Sequential Greedy Method
function solveDistributedSequentialGreedyBtnFcn(){
	
	consolePrint("Solving Using Distributed - Sequential Greedy Algorithm: Started. Please wait...");
	// save the existing particle shadows
	savedParticles = particleShadows;

	// remove all the agents
	particleShadows = [];particles = [];

	
	savedParameters[0] = []; // chosen candidate indexes (by each machine)
	savedParameters[1] = 0;  // length of current particle shadows (agents added so far)
	savedParameters[2] = 0;  // number of computations did (to check O(nN))
	savedParameters[3] = []; // set of reduced candidates for each machine (for display purposes)


	savedParameters[4] = 0; // current agent number-same as savedParameters[1], thus no use
	//savedParameters[5] = machineAssignments;
	savedParameters[5] = [];
	// the first agent should see the first grid point from its location.
	var reducedIndexes = [];
	var reducedCandidatePoints = [];
	for(var i=0;i<submodularityCandidates.length;i++){
		if(distP2(submodularityCandidates[0],submodularityCandidates[i])<=senRange){
			if(isLineOfSight(submodularityCandidates[0],submodularityCandidates[i])){
				reducedIndexes.push(i);
				reducedCandidatePoints.push(submodularityCandidates[i]);
			}
		}
	}	
	savedParameters[3][0] = reducedCandidatePoints;
	savedParameters[5][0] = reducedIndexes;
	
	savedParameters[6] = []; //objective function value of each machine


	// now we need to calculate greedy algorithm (centralized general version) one by one
	submodularityMode = 6;


	
}

function iterationOfDistributedSequentialGreedy(currentMachine){
	
	var reducedIndexes = savedParameters[5][currentMachine];

	// normal greedy
	var chosenCandidateIndexes = savedParameters[0];
	
	var costArray = [];
	var computations = 0;
	for(var i = 0; i<reducedIndexes.length; i++){// scanning candidates
		
		var j = reducedIndexes[i];

		if(chosenCandidateIndexes.includes(j)&&!(reducedIndexes.length<=chosenCandidateIndexes.length)){// not choses previously
			costArray[i] = 0; // will disencourage chosing this point
		}else{
			addAgentToPoint(submodularityCandidates[j].x,submodularityCandidates[j].y);
			
			// the following line  can be improved by considering only the cost increment of adding an agent 
			//costArray[j] = Math.round(globalObjective()*1000)/1000;
			// improved version: cost increment is only calculated using local info.
			costArray[i] = Math.round(particleShadows[particleShadows.length-1].localObjectiveFunction()*1000)/1000;
			
			removeAgent(); // remove last element
			computations++;
		} 

	}
	var chosenCandidateIndex = costArray.indexOf(Math.max.apply(Math,costArray));
	chosenCandidateIndex = reducedIndexes[chosenCandidateIndex];
	var solutionPoint = submodularityCandidates[chosenCandidateIndex];

	//print(chosenCandidateIndex,solutionPoint);

	addAgentToPoint(solutionPoint.x,solutionPoint.y);
	chosenCandidateIndexes.push(chosenCandidateIndex);
	savedParameters[0] = chosenCandidateIndexes;
	savedParameters[1] = savedParameters[1]+1;
	savedParameters[2] = savedParameters[2]+computations;

	// need to update the available strategy list for the next agent:
	var reducedIndexes = [];
	var reducedCandidatePoints = [];
	for(var i=0;i<submodularityCandidates.length;i++){
		for(var j=0;j<chosenCandidateIndexes.length;j++){
			var s1 = submodularityCandidates[chosenCandidateIndexes[j]];
			var s2 = submodularityCandidates[i];
			var breakFromThis = false;
			if(distP2(s1,s2)<2*senRange){
				if(isLineOfSight(s1,s2)){
					reducedIndexes.push(i);
					reducedCandidatePoints.push(submodularityCandidates[i]);
					breakFromThis = true;// no need to see whether other agents can see the same point
				}else{
					//skewed connections can happen-lets ignore that for now
				}
			}
			if(breakFromThis){
				break;
			}
		}
	}	
	savedParameters[3][currentMachine+1] = reducedCandidatePoints;
	savedParameters[5][currentMachine+1] = reducedIndexes;
}




// End - Distributed Sequential Greedy Method 


function deployAgentsBtnFcn(){
	submodularityMode = 0;
}





































/////////////////////////////////// Lipschitz
function tuneLipschitzConstants(){
	consolePrint("Start Tuning Lipschitz Constant K1 - Please Wait...");

	// storing initial agent positions
	var agentInitialPositions = [];
	var agentInitialModes = [];
	for(var i=0;i<particleShadows.length;i++){
		agentInitialPositions.push(particleShadows[i].position);
		agentInitialModes.push(particleShadows[i].isBoostingActivated);
	}


	var numberOfAgents = particleShadows.length; //N

	var numberOfParticlesForEstimateK1 = 100;

	var particlesForEstimateK1 = []; // array of arrays (array of particles (particle = array of candidate agents))

	var particleForEstimateK1 = []; // sample particle

	

	for(var i = 0; i<numberOfParticlesForEstimateK1; i++){

		particleForEstimateK1 = [];
		
		for(var j = 0; j<numberOfAgents; j++){
			
			cordinateX = Math.round(width*Math.random());
			cordinateY = Math.round(height*Math.random());
			samplePoint = new Point2(cordinateX,cordinateY)
			
			if(getEventDensity(samplePoint)>0){//valid sample point
				particleForEstimateK1[j] = new Particle(cordinateX,cordinateY);
			}else{
				j--; // try kagain
			}
		}

		if (particleForEstimateK1.length==numberOfAgents){
			particlesForEstimateK1.push(particleForEstimateK1); //new particle added
		}else{
			print("Generation Error");
		}
	}

	// pick two particles say S1 and  S2
	// get the distace between then ||S1-S2||
	var numOfPairs = 500;
	var K1Array = [];
	var K1LowerBound;

	for(var trials = 0; trials<numOfPairs; trials++){

		var index1 = Math.floor(Math.random() * numberOfParticlesForEstimateK1);
		var index2 = Math.floor(Math.random() * numberOfParticlesForEstimateK1);
		while(index1==index2){
			index2 = Math.floor(Math.random() * numberOfParticlesForEstimateK1);
		}

		var S1 = particlesForEstimateK1[index1];
		var S2 = particlesForEstimateK1[index2];


		var distS1S2 = 0;
		for(var agentNum=0; agentNum<numberOfAgents; agentNum++){
			distS1S2 = distS1S2 + sq(S1[agentNum].position.x - S2[agentNum].position.x)+sq(S1[agentNum].position.y - S2[agentNum].position.y);
		} 
		distS1S2 = sqrt(distS1S2);

		
		
		for(var i = 0; i<numberOfAgents; i++){ //for Hi

			resetParticleShadowPositions(S1);
			var distDHiS1_DHiS2 = 0;
			var d1 = [];
			var d2 = [];
			

			for(var j=0;j<numberOfAgents;j++){
				if (j==i) {
					d1[j] = S1[i].getDerivativesWithBoosting();
				}else{
					d1[j] = S1[i].getJointDerivative(j,0);
				}
			} 

			resetParticleShadowPositions(S2);

			for(var j=0;j<numberOfAgents;j++){
				if (j==i) {
					//d1 = S1[i].getDerivativesWithBoosting();
					d2[j] = S2[i].getDerivativesWithBoosting();
					distDHiS1_DHiS2 = distDHiS1_DHiS2 + sq(d1[j][0]-d2[j][0]) + sq(d1[j][1]-d2[j][1]);
				}else{
					//d1 = S1[i].getJointDerivative(j,0);
					d2[j] = S2[i].getJointDerivative(j,0);
					distDHiS1_DHiS2 = distDHiS1_DHiS2 + sq(d1[j][0]-d2[j][0]) + sq(d1[j][1]-d2[j][1]);
				}
			} 


			distDHiS1_DHiS2 = sqrt(distDHiS1_DHiS2);

			//print(distS1S2);
			//print(distDHiS1_DHiS2);
			K1LowerBound = round(1000*distDHiS1_DHiS2/distS1S2)/1000;
			//print(K1LowerBound);
			K1Array.push(K1LowerBound);

		}
	}
	//print(K1Array);
	lipschitzConstantK1 = max(K1Array);
	for(var i = 0; i<particleShadows.length; i++){
		particleShadows[i].lipschitzConstantK1 = lipschitzConstantK1;
	}
	//print(lipschitzConstantK1);

	//reset();
	resetParticleShadowPositions(agentInitialPositions);
	setModeOfAgents(particleShadows,agentInitialModes);




	consolePrint("Lipschitz coeffcient K1 = "+lipschitzConstantK1+" was decided out of "+K1Array.length+" trials")
	return(lipschitzConstantK1);
}


function findObstacleIntersectingPoints(agent){

	var s = agent.position; 
	var R = agent.senRange;

	var intersectingPoints = [];

	// checking intersection with obstacle boundaries (Convergence18 - note)
	for(var j=0;j<obstacles.length;j++){
		for(v=0;v<obstacles[j].vertices.length;v++){
			
			if(v==0){
				x1 = obstacles[j].vertices[obstacles[j].vertices.length-1];
				x2 = obstacles[j].vertices[v]; 
			}else{
				x1 = obstacles[j].vertices[v-1];
				x2 = obstacles[j].vertices[v];
			}

			var distX1X2 = distP2(x1,x2);
			var d = abs((x2.y-x1.y)*s.x-(x2.x-x1.x)*s.y+x2.x*x1.y-x2.y*x1.x);
			d = d/distX1X2;

			if(d < R){
				if(x2.x==x1.x){// cirlse intersection with lines of type x=c
					// equation of the line : x = c
					var c = x1.x;
					var x1Hat = c;
					var x2Hat = c;

					var y1Hat = s.y + sqrt(sq(R)-sq(c-s.x));
					var y2Hat = s.y - sqrt(sq(R)-sq(c-s.x));

					if(min(x1.y,x2.y) <= y1Hat && max(x1.y,x2.y) >= y1Hat){
						intersectingPoints.push(new Point2(x1Hat,y1Hat));
					}
					
					if(min(x1.y,x2.y) <= y2Hat && max(x1.y,x2.y) >= y2Hat){ 
						intersectingPoints.push(new Point2(x2Hat,y2Hat));
					}
					
				}
				else if(x2.y==x1.y){// cirlse intersection with lines of type y=c
					// equation of the line : y = c
					var c = x1.y;
					var y1Hat = c;
					var y2Hat = c;

					var x1Hat = s.x + sqrt(sq(R)-sq(c-s.y));
					var x2Hat = s.x - sqrt(sq(R)-sq(c-s.y));

					if(min(x1.x,x2.x) <= x1Hat && max(x1.x,x2.x) >= x1Hat){
						intersectingPoints.push(new Point2(x1Hat,y1Hat));
					}
					
					if(min(x1.x,x2.x) <= x2Hat && max(x1.x,x2.x) >= x2Hat){
						intersectingPoints.push(new Point2(x2Hat,y2Hat));
					}
					
				}else{// m not equal to 0
					// equation of the line
					var m = (x2.y-x1.y)/(x2.x-x1.x);
					var c = x1.y - x1.x*m;

					var a = (1+sq(m));
					var b = (-2*s.x + 2*m*c - 2*m*s.y);
					var c1 = sq(s.x)+sq(c)+sq(s.y) - 2*c*s.y - sq(R);

					if((sq(b)-4*a*c1)>0){
						var Del = sqrt(sq(b)-4*a*c1);
						var x1Hat = (-b+Del)/(2*a);
						var x2Hat = (-b-Del)/(2*a);
						var y1Hat = m*x1Hat + c;
						var y2Hat = m*x2Hat + c;

						var lambda1 = (x1Hat-x2.x)/(x1.x-x2.x);
						var lambda2 = (x2Hat-x2.x)/(x1.x-x2.x);

						if(lambda1>=0 && lambda1<=1){
							intersectingPoints.push(new Point2(x1Hat,y1Hat));
						}

						if(lambda2>=0 && lambda2<=1){
							intersectingPoints.push(new Point2(x2Hat,y2Hat));
						}

					}else{
						print("intersectionError");
					}

				}

			}
		}
		

	}

	for(v=0;v<boundaryObstacle.vertices.length;v++){
			
		if(v==0){
			x1 = boundaryObstacle.vertices[boundaryObstacle.vertices.length-1];
			x2 = boundaryObstacle.vertices[v]; 
		}else{
			x1 = boundaryObstacle.vertices[v-1];
			x2 = boundaryObstacle.vertices[v];
		}

		var distX1X2 = distP2(x1,x2);
		var d = abs((x2.y-x1.y)*s.x-(x2.x-x1.x)*s.y+x2.x*x1.y-x2.y*x1.x);
		d = d/distX1X2;

		if(d < (R)){
			if(x2.x==x1.x){// cirlse intersection with lines of type x=c
				// equation of the line : x = c
				var c = x1.x;
				var x1Hat = c;
				var x2Hat = c;

				var y1Hat = s.y + sqrt(sq(R)-sq(c-s.x));
				var y2Hat = s.y - sqrt(sq(R)-sq(c-s.x));

				if(min(x1.y,x2.y) <= y1Hat && max(x1.y,x2.y) >= y1Hat){
					intersectingPoints.push(new Point2(x1Hat,y1Hat));
				}
				
				if(min(x1.y,x2.y) <= y2Hat && max(x1.y,x2.y) >= y2Hat){ 
					intersectingPoints.push(new Point2(x2Hat,y2Hat));
				}
				
			}
			else if(x2.y==x1.y){// cirlse intersection with lines of type y=c
				// equation of the line : y = c
				var c = x1.y;
				var y1Hat = c;
				var y2Hat = c;

				var x1Hat = s.x + sqrt(sq(R)-sq(c-s.y));
				var x2Hat = s.x - sqrt(sq(R)-sq(c-s.y));

				if(min(x1.x,x2.x) <= x1Hat && max(x1.x,x2.x) >= x1Hat){
					intersectingPoints.push(new Point2(x1Hat,y1Hat));
				}
				
				if(min(x1.x,x2.x) <= x2Hat && max(x1.x,x2.x) >= x2Hat){
					intersectingPoints.push(new Point2(x2Hat,y2Hat));
				}
				
			}else{// m not equal to 0
				// equation of the line
				var m = (x2.y-x1.y)/(x2.x-x1.x);
				var c = x1.y - x1.x*m;

				var a = (1+sq(m));
				var b = (-2*s.x + 2*m*c - 2*m*s.y);
				var c1 = sq(s.x)+sq(c)+sq(s.y) - 2*c*s.y - sq(R);

				if((sq(b)-4*a*c1)>0){
					var Del = sqrt(sq(b)-4*a*c1);
					var x1Hat = (-b+Del)/(2*a);
					var x2Hat = (-b-Del)/(2*a);
					var y1Hat = m*x1Hat + c;
					var y2Hat = m*x2Hat + c;

					var lambda1 = (x1Hat-x2.x)/(x1.x-x2.x);
					var lambda2 = (x2Hat-x2.x)/(x1.x-x2.x);

					if(lambda1>=0 && lambda1<=1){
						intersectingPoints.push(new Point2(x1Hat,y1Hat));
					}

					if(lambda2>=0 && lambda2<=1){
						intersectingPoints.push(new Point2(x2Hat,y2Hat));
					}

				}else{
					print("intersectionError");
				}

			}

		}
	}

	//print(intersectingPoints);

	// line of sight check
	var intersectingPointsFinal = [];
	for(var i=0; i<intersectingPoints.length; i++){
		////var intermediatePoint = intermediateP2(agentPosition,intersectingPoints[i],0.05);//~=1-(5/200)
		////if(isLineOfSight(agentPosition,intermediatePoint)){
			////intersectingPointsFinal.push(intersectingPoints[i]);
		////}
		var normalDirection = normalizeP2(minusP2(intersectingPoints[i],agent.position));
        var normalDirection1 = new Point2(-normalDirection.y,normalDirection.x);
        var normalDirection2 = new Point2(normalDirection.y,-normalDirection.x);
        var allignDirection = normalizeP2(minusP2(agent.position,intersectingPoints[i]));
        var Vj1 = plusP2(intersectingPoints[i],productP2(normalDirection1,2));
        var Vj2 = plusP2(intersectingPoints[i],productP2(normalDirection2,2));
        var Vj3 = plusP2(intersectingPoints[i],productP2(allignDirection,2));
		
		var inSensing1 = !isOutOfCanvas(Vj1) && isLineOfSight(agent.position,Vj1);
		var inSensing2 = !isOutOfCanvas(Vj2) && isLineOfSight(agent.position,Vj2);
		var inSensing3 = !isOutOfCanvas(Vj3) && isLineOfSight(agent.position,Vj3);

        if( ((!inSensing1)&&inSensing2 || inSensing1&&(!inSensing2)) ){ //&&inSensing3
        	intersectingPointsFinal.push(intersectingPoints[i]);
        }
	}

	if(intersectingPointsFinal.length%2==1){
		//print("Error0");
		//print(intersectingPoints);
	}

	return intersectingPointsFinal;


}





