function Event() {
	
}

function getEventDensity(interestedPoint){
	//if(max([interestedPoint.x,interestedPoint.y])>600||min([interestedPoint.x,interestedPoint.y])<0){
	if(isOutOfCanvas(interestedPoint)){
		return 0;
	}
	else if(obstacles.length > 0 && isColorEqualP2C(interestedPoint,obstacleColor)){
		return 0;//point is on top of an obstacle
	}
	else{
		return 1;
	}
		
}

