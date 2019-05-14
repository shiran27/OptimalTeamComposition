function Point2(x,y) {

	this.x = x;
	this.y = y; 
	
	this.lengthP2 = function(){
		var val = Math.sqrt(this.x*this.x + this.y*this.y);
		return val;
	}
	

	this.setP2 = function(x,y){
		this.x = x;
		this.y = y;
	}

	this.copyP2 = function(a){
		this.x = a.x;
		this.y = a.y;
	}

	this.shiftP2 = function(delta_x, delta_y){
		this.x += delta_x;
		this.y += delta_y;
	}

	this.cloneP2 = function(){
		return new Point2(this.x,this.y);
	}
	
	this.colorP2 = function(){
		return get(this.x,this.y);
	}
	
	this.pixelColorP2 = function(){
		var offset = (Math.round(this.y)*width + Math.round(this.x)) * 4; 
		//print(pixelMap.length)
		return [ pixelMap[offset], pixelMap[offset + 1], pixelMap[offset + 2], pixelMap[offset + 3] ]; 
	}	

}

//print points

function printPointArrayP2(pointArray,colorIn,r){
	
	//fill(128,0,0);
	fill(colorIn);
	stroke(colorIn);
	//stroke(128,0,0);
	for(var i=0 ; i<pointArray.length; i++){
	
		ellipse(pointArray[i].x, pointArray[i].y, r, r);
		
	}
	
	
}


//function to find whether a given point is inside or outside the canvas

function isOutOfCanvas(interestedPoint){
	
	if(width==height){	
		return (Math.max(interestedPoint.x,interestedPoint.y) > width || Math.min(interestedPoint.x,interestedPoint.y) < 0);
	}
	else{
		return ( (interestedPoint.x > width || interestedPoint.y > height) || Math.min(interestedPoint.x,interestedPoint.y) < 0 );
	}
	
}


// function to find whether the point is way way out of the canvas (intersectiong two edges)
function isWayOutOfCanvas(nextPosition){
	var guard = 20;
	if(nextPosition.x < 0 && nextPosition.y < 0){
		return [true, new Point2(guard,guard)];
	}
	else if(nextPosition.x < 0 && nextPosition.y > height){
		return [true, new Point2(guard,height-guard)];
	}
	else if(nextPosition.x > width && nextPosition.y > height){
		return [true, new Point2(width-guard,height-guard)];
	}
	else if(nextPosition.x > width && nextPosition.y < 0){
		return [true, new Point2(width-guard,guard)];
	}
	else {
		return [false, nextPosition];
			
	}

}






// extending from point in the given direction till end of sensor range
function pointsBeyondP2(particle,b,stepSize){//use senRange to get z
	var a = particle.position;

	var unitVector = normalizeP2(minusP2(b,a));
	var pointArray = [];
	var pointArrayTemp = [];
	
	var z = plusP2(a,productP2(unitVector,particle.senRange));
	var numOfSteps = Math.ceil(distP2(b,z)/stepSize);
	
	for(var i=0; i<numOfSteps; i++){
		pointArray.push(plusP2(b,productP2(unitVector,stepSize*(i+1))));
	}
	pointArray[pointArray.length-1] = z;//last element should be z

	//fwd check
	for(var i=0; i<numOfSteps; i++ ){
		var z_temp = pointArray[i];
		if(i<10){
			pointArrayTemp.push(pointArray[i]);
			//print("added");
		}
		else if(isOutOfCanvas(z_temp)){
			break;
		}
		else if(isColorEqualP2C(z_temp,obstacleColor)){
			for (var j = pointArrayTemp.length-1; j>=0; j--){//foward search
				if(isColorEqualP2C(pointArrayTemp[j],obstacleColor)){
					pointArrayTemp.pop();
				}
				else{
					break
				}
			}
			break;
		}
		else{
			pointArrayTemp.push(pointArray[i]);
		}
				
			
	}

	pointArray = pointArrayTemp;

	return pointArray;
	
	//check z feasibility
}

//extending from point in the given direction
function pointsInteriorP2(a,b,stepSize){
	
	var unitVector = normalizeP2(minusP2(b,a));
	var pointArray = [];
	var pointArrayTemp = [];
	
	var numOfSteps = Math.ceil(distP2(a,b)/stepSize);
	
	for(var i=0; i<numOfSteps; i++){
		pointArray.push(plusP2(a,productP2(unitVector,stepSize*(i+1))));
	}
	
	pointArray[pointArray.length-1] = b;
	
	for (var j = 0; j<pointArray.length; j++){//foward search
		if(!isColorEqualP2(b,pointArray[j])){
			pointArrayTemp.push(pointArray[j]);
		}
		else{
			break;
		}
	}
	
	
	return pointArrayTemp;
}


function getInteriorTrajectoryPointsP2(a,b,stepSize){//negilecting obstacles
	var unitVector = normalizeP2(minusP2(b,a));
	var pointArray = [];
		
	var numOfSteps = Math.ceil(distP2(a,b)/stepSize);
	
	for(var i=0; i<numOfSteps; i++){
		pointArray.push(plusP2(a,productP2(unitVector,stepSize*(i+1))));
	}
	
	pointArray[pointArray.length-1] = b;
	
	return pointArray
}



//pixel color matching between two points in the space
function isColorEqualP2(a,b){
	var col1 = a.pixelColorP2();
	var col2 = b.pixelColorP2();
	var count = 0;
	for(var i=0; i<col1.length; i++){
		if (col1[i]==col2[i]){
			count++;
		}
	}
	if(count==4){
		return true;
	}
	else{
		return false;
	}
}

//pixel color matching between a point ("a") and a mentioned color("b")
function isColorEqualP2C(a,b){
	//var col1 = a.colorP2();
	var col1 = a.pixelColorP2();
	var col2 = b.levels;
	var count = 0;
	for(var i=0; i<col1.length; i++){
		if (col1[i]==col2[i]){
			count++;
		}
	}
	if(count==4){
		return true;
	}
	else{
		return false;
	}
}


function isColorEqualP2CRigour(a,b){
	
	var aArray = [];
	var gridSize = 3;

	for(var x = -1; x<2; x++){
		for(var y = -1; y<2; y++){
			//print(x,y);
			var interestedPoint = new Point2(a.x+x*gridSize,a.y+y*gridSize);
			aArray.push(interestedPoint);
		}

	}
	//print(aArray);
	var countTotal = 0;

	for(var j=0; j<aArray.length; j++){
		
		if(isColorEqualP2C(aArray[j],b)){
			countTotal++;
		}

	}

	//print(countTotal)
	if(countTotal>=1){
		return true;
	}else{
		return false;
	}
}




//returns inermediate or extending point from b as a percentage of ||a-b||
//returns length(point2-point1)
function intermediateP2(a,b,lambda){
	//there is a gray area near vertices - 
	//var resultPoint = new Point2(lambda*a.x+(1-lambda)*b.x,lambda*a.y+(1-lambda)*b.y);
	//if (Math.floor(resultPoint.x)==b.x && Math.floor(resultPoint.y)==b.y){
		//print("hell");
	//}
	return new Point2(lambda*a.x+(1-lambda)*b.x,lambda*a.y+(1-lambda)*b.y);
	
}

function distP2(a,b){
	return Math.sqrt(Math.pow((b.x-a.x),2)+Math.pow((b.y-a.y),2));
}

//returns (point1+point2)/2
function middlePointP2(a,b){
	return new Point2((a.x+b.x)/2,(a.y+b.y)/2);
}


//returns point1/norm(point1)
function normalizeP2(a) {
    var foo = 1 / a.lengthP2();
    return new Point2(a.x*foo,a.y*foo);
}

//returns point2+point2
function plusP2(a, b){
    return new Point2(a.x+b.x,a.y+b.y);
}


//returns point2-point2
function minusP2(a, b){
    return new Point2(a.x-b.x,a.y-b.y);
}


//returns point2*double
function productP2(a, b){
    return new Point2(a.x*b,a.y*b);
}

//returns point2/double
function divideP2(a, b){
    return new Point2(a.x/b,a.y/b);
}

//dot product
function dotP2(a, b){
    return a.x * b.x + a.y * b.y;
}

//Norm
function normP2(a){
    return sqrt(sq(a.x)+sq(a.y));
}

//cross product
function crossP2(a, b){
    return a.x*b.y-b.x*a.y;
}

//angle of line between two point
function atan2P2(a, b){
    return Math.atan2(b.y-a.y,b.x-a.x);
}

//returns point1 == point2
function isEqualP2(a, b){
    if (a.x==b.x && a.y==b.y){
            return true;
    }
    else{
            return false;
    }
}

//returns rotated vector - counter clockwise
function rotateP2(a, angle){
	return new Point2(Math.cos(angle)*a.x-Math.sin(angle)*a.y,Math.cos(angle)*a.y+Math.sin(angle)*a.x);
}


function saturateP2(a,aMaxSq){
	if(dotP2(a, a) >= aMaxSq){ //a'a<aMaxSq
		return productP2(a,sqrt(aMaxSq/dotP2(a,a)));
	}
	else{
		return a;
	}
}

function avoidEscapeP2(a){
	var b = new Point2(a.x,a.y);
	var gurad = 5;

	if(a.x > width){
		b.x = width-gurad;
	}else if(a.x < 0){
		b.x = gurad;
	}
	if(a.y > height){
		b.y = height-gurad;
	}else if(a.y<0){
		b.y = gurad;
	}
	return b;

}

function repeatP2(a,N){
	result = [];
	for(i=0;i<N;i++){
		append(result,a);
	}
	return result;
}

/* if(isLineOfSight(s,intermediateP2(s,v_ij,0.3))){
                        print(intermediateP2(s,v_ij,0.3))
                        printPointArrayP2([intermediateP2(s,v_ij,0.5)],"blue");
            			if(!isColorEqualP2C(intermediateP2(s,v_ij,-0.3), obstacleColor)){
            				if(!isColorEqualP2C(intermediateP2(s,v_ij,0.3),obstacleColor)){*/

function checkNearPixels(s,v,d){
	var unitIncrement = productP2(normalizeP2(minusP2(s,v)),d);
	
	if(unitIncrement.x<0){
		unitIncrement.x = floor(unitIncrement.x);
	}else{
		unitIncrement.x = ceil(unitIncrement.x);
	}

	if(unitIncrement.y<0){
		unitIncrement.y = floor(unitIncrement.y);
	}else{
		unitIncrement.y = ceil(unitIncrement.y);
	}

	var z1 = plusP2(v,unitIncrement);
	//print(z1);
	//print(z1.colorP2());
	var z2 = minusP2(v,unitIncrement);
	//print(z2);
	//print(z2.colorP2());
	if(isLineOfSight(s,z1)){
		if(!isColorEqualP2C(z2, obstacleColor)){
			return true;
		}
		else{
			return false
		}
	}
	else{
		//print(v.x+","+v.y+" is not line of sight")
		return false;
	}

}

function drawArrow(x1,x2,r,textMassage,textOffset){
	var offset = r;
	strokeWeight(2);
	stroke(0);
    ellipse(x1.x, x1.y, r, r); //starting vertex
    //ellipse(x2.x, x2.y, r, r); //ending vertex
    line(x1.x, x1.y, x2.x, x2.y); //draw a line beetween the vertices
	var unitVector = normalizeP2(minusP2(x2,x1));
    var x3 = plusP2(x2,productP2(unitVector,textOffset));
    stroke(100,0,0);
    strokeWeight(1);
    text(textMassage, x3.x, x3.y);
    stroke(0);
    // this code is to make the arrow point
    push() //start new drawing state
    var angle = atan2(x1.y - x2.y, x1.x - x2.x); //gets the angle of the line
    translate(x2.x, x2.y); //translates to the destination vertex
    rotate(angle-HALF_PI); //rotates the arrow point
    triangle(-offset*0.5, offset, offset*0.5, offset, 0, -offset/2); //draws the arrow point as a triangle
    pop();
}

function getIntersection(E,C,A,B){// AB is the edge; E is the current position; C is the next position
	var determ = (E.x-C.x)*(A.y-B.y)-(E.y-C.y)*(A.x-B.x);
	var numX = (E.x*C.y-E.y*C.x)*(A.x-B.x) - (E.x-C.x)*(A.x*B.y-A.y*B.x);
	var numY = (E.x*C.y-E.y*C.x)*(A.y-B.y) - (E.y-C.y)*(A.x*B.y-A.y*B.x);

	if(Math.abs(determ)>0.001){
		return new Point2(numX/determ,numY/determ);
	}
	else{
		print("intersection error "+determ);
		print(E,C,A,B);
		return E;
	}
}

function getProjection(E,A,B,C){ // AB is the edge and E is the agent position and C is the next position
	var guard = 5; // alpha
	// 1:A; 2:B; bar:C; hat:E; tilde:D; 0:F 

	// lets first dind the projection of C on to AB line (finding point D)
	var AB = distP2(A,B);

	var tildet = ((A.y - B.y)*(C.x - B.x) - (A.x - B.x)*(C.y - B.y))/(pow(AB,2));
	var D = new Point2( (B.y - A.y)*tildet + C.x , (A.x-B.x)*tildet + C.y );
	////print("D0: "+round(D.x),round(D.y));
	// if D is on  middle of the AB??
	if(abs((AB+distP2(B,D))-distP2(A,D))<0.001){
		var tempPoint = plusP2(B,productP2(normalizeP2(minusP2(B,A)),guard));
		if(isOutOfCanvas(tempPoint)){
			D = B;
		}else if(!isLineOfSight(E,tempPoint)){
			D = B;
		}else{
			D = tempPoint;
		}
	}
	else if(abs((AB+distP2(A,D))>=distP2(B,D))<0.001){
		var tempPoint = plusP2(A,productP2(normalizeP2(minusP2(A,B)),guard));
		if(isOutOfCanvas(tempPoint)){
			D = A;
		}else if(!isLineOfSight(E,tempPoint)){
			D = A;
		}else{
			D = tempPoint;
		}
	}

	////print("D: "+round(D.x),round(D.y));
	// solving for F

	

	/*var alpha1 = (B.y - A.y)*(E.x - D.x) - (B.x - A.x)*(E.y - D.y);
	var beta1 = (B.y - A.y)*D.x - (B.x - A.x)*D.y + B.x*A.y - B.y*A.x;

	var m1 = (guard*AB - beta1)/alpha1;
	var m2 = -1*(guard*AB + beta1)/alpha1;
	var m;


	var F1 = new Point2( (E.x - D.x)*m1 + D.x , (E.y - D.y)*m1 + D.y );
	var F2 = new Point2( (E.x - D.x)*m2 + D.x , (E.y - D.y)*m2 + D.y );
	var F;
	print("F1: "+round(F1.x),round(F1.y));
	print("F2: "+round(F2.x),round(F2.y));
	if(distP2(E,F1)<distP2(E,F2)){
		F = F1;
		//F1 = F2
	}
	else{
		F = F2;
		//F1=F1;
	}*/

	var F = D;
	var F1 =  plusP2(F,productP2(normalizeP2(minusP2(F,E)),guard));


	if(isOutOfCanvas(F)){
		F = avoidEscapeP2(F);
	}
	if(isColorEqualP2C(D,obstacleColor)||isColorEqualP2C(F1,obstacleColor)||isColorEqualP2C(F,obstacleColor)){
		F = plusP2(F,productP2(normalizeP2(minusP2(E,F)),guard));
		if(isOutOfCanvas(F)){
			F = avoidEscapeP2(F);
		}
	}
	////print("F: "+round(F.x),round(F.y));
	return F;

}