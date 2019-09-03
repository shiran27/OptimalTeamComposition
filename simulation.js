//This function calculates the joint probability of detection at each point


function detectionProbabilityGlobal(interestedPoint){
    var jointMissProbability = 1;
    
    if(particleShadows.length==0){//no particleShadows - nothing to detect
        return 0;
    }
    else{
        for (var i = 0; i < particleShadows.length; i++){
        	//print(particles[i].sensingModelFunction(interestedPoint));
            jointMissProbability = jointMissProbability*(1-particleShadows[i].sensingModelFunction(interestedPoint));
        }
    }
    return (1-jointMissProbability);
}

//global objective function
function globalObjective(){//old method used to calculate the objective function
    var globalObjectiveValue = 0;
    var stepSize = descretizationLevel; 
    var halfStepSize = stepSize/2;
    var areaFactor = sq(stepSize);
    
    for (var x = halfStepSize; x <= width - halfStepSize; x+=stepSize){
        for(var y = halfStepSize; y <= height - halfStepSize; y+=stepSize){
        	
        	var interestedPoint = new Point2(x,y);
            var eventDensity = getEventDensity(interestedPoint);
            
            if(eventDensity>0){
            	//print(detectionProbabilityGlobal(interestedPoint));
            	globalObjectiveValue = globalObjectiveValue + detectionProbabilityGlobal(interestedPoint)*eventDensity*areaFactor;
            }
            
        }
    }
    return globalObjectiveValue;
}



//line of sight between two points
//bounding box for convex obstacles
//sampling for non-convex

function isLineOfSight(a,b){
	
	//quick check
	//print(isColorEqualP2C(b,obstacleColor));
	
	//var startTime5 = millis();

	if(obstacles.length>0 && isColorEqualP2C(b,obstacleColor)){
		return false;
		
	}
	
	//print(Math.round(millis()-startTime5));
	
	if(obstacles.length==0){//if no obstacles - everything can be seen
		return true;
	}
	
	//rigorous check stage 1

	var normalVector = new Point2(-(b.y-a.y),b.x-a.x);
	var offset = dotP2(a,normalVector);
	
	//print(normalVector);
	var count = 0 ; //=1 if obstacle not intersected
	var count2 = 0; //number of edges which does not intersect

	for(var i = 0; i<obstacles.length; i++){
		if(obstacles[i].isConvex){
			
			count = 0; //=1 if this obstacle does not intersect
			//bounding box
			if(a.x > obstacles[i].largestX && b.x > obstacles[i].largestX){count++;}
			else if(a.y > obstacles[i].largestY && b.y > obstacles[i].largestY){count++;}
			else if(a.x < obstacles[i].smallestX && b.x < obstacles[i].smallestX){count++;}
			else if(a.y < obstacles[i].smallestY && b.y < obstacles[i].smallestY){count++;}
			else{//projection check: //rigorous check stage 2
				// checking edge by edge for an intersection
				
				for(var j = 0; j < obstacles[i].vertices.length-1; j++){
					count2 = 0;//num of edges which does not intersect
					//bounding box
					if(a.x > obstacles[i].largestXArray[j] && b.x > obstacles[i].largestXArray[j]){count2++;}
					else if(a.y > obstacles[i].largestYArray[j] && b.y > obstacles[i].largestYArray[j]){count2++;}
					else if(a.x < obstacles[i].smallestXArray[j] && b.x < obstacles[i].smallestXArray[j]){count2++;}
					else if(a.y < obstacles[i].smallestYArray[j] && b.y < obstacles[i].smallestYArray[j]){count2++;}
					else{//projection check2: // rigorous check stage 3
						//print("here")
						//projection 1
						var projections1 = [];
						projections1.push(dotP2(obstacles[i].vertices[j],normalVector)-offset);
						projections1.push(dotP2(obstacles[i].vertices[j+1],normalVector)-offset);
						
						//projection 2
						var projections2 = [];
						projections2.push(dotP2(a,obstacles[i].normalDirectionArray[j])-obstacles[i].offsetArray[j]);
						projections2.push(dotP2(b,obstacles[i].normalDirectionArray[j])-obstacles[i].offsetArray[j]);
						
						//print(projections1);
						if(projections1[0]*projections1[1]>0 || projections2[0]*projections2[1]>0 ){count2++;}
					}
					
					if(count2 == 0){//intersection found with a edge (obs-i,edge-j) - stop
						return false;
					}					
				}
				
				if(count2 == 1){
					count++;
				}
				
			}
			if(count == 0){//intersection with ith obs
				return false;
			}
		}
		else{
			if(addObstacleMode==0){
				print("Non Convex Obstacle Exists!!!");
			}
			//obstacles[i].calculateReflexVertices();
			//print(obstacles[i].isConvex);
			//sampling based tech for nonConvex obstacles 
		}
	}
	
	if(count==1){//no intersection with all the obstacles till the last one
		//print("bfbd")
		return true;
	}
	else{
		return false;
	}
}


function drawSensingColorMap(pixelSize){ 
	
	if(displayCoverageDensity.checked){

		for (var i = 0; i < width; i += pixelSize){

			for (var j = 0; j < height; j += pixelSize){
				colorMode(RGB, 255, 255, 255, 1);
				noStroke();
				
				////if(obstacles.length>0){
					detectionProbability = detectionProbabilityGlobal(new Point2(i+pixelSize/2,j+pixelSize/2));
					//fill(0, 0, 150, detectionProbability*2);
				////}
				////else{
				////	detectionProbability = sensing(i,j,senRange);
					//fill(0, 0, 150, detectionProbability);
				////}
                
                if (detectionProbability > 0.95) {
                    fill(250,0, 255);
                } else if (detectionProbability > 0.5) {
                    fill(0,255,0,detectionProbability);
                } else {
                    fill(255,210,0, detectionProbability*2);
                }
				
				
				
				rect(i,j,pixelSize,pixelSize);

			}

		}

	}
}



/*
var costMatrix = formCostMatrix();
var assignments = solveAssignmentProblem(costMatrix);*/

function formCostMatrix(){
	//use distances among particles and particleShadows to form distance (as a cost) Matrix
	var costMatrix = [];
	var N = particles.length;
	for(var i=0; i<N; i++){
		append(costMatrix,[]);
		for(var j=0; j<N; j++){
			costMatrix[i][j] = round(distP2(particles[i].position,particleShadows[j].position));
		}
	}
	return costMatrix;	
}

function consolePrint(consoleText){
	/*var h4Item = document.getElementById("consoleText");
	h4Item.innerHTML += ">> "+consoleText+"<br>";
	h4Item.scrollTop = h4Item.scrollHeight;*/

	document.getElementById("consoleText").innerHTML += ">> "+consoleText+"<br>";
	document.getElementById("consoleText").scrollTop = document.getElementById("consoleText").scrollHeight;
	//document.getElementById("consoleText").innerHTML = consoleText;
}



// for the network flow problem
function formSimplexTableu(){

	
	
	var variables = [];
	var objectiveFunction = [];
	var edgeCost = 0; //length
	var variableCount = 0;

	// edges between particles and particleShadows
	for(var i = 0; i<particles.length; i++){
		for(var j = 0; j<particleShadows.length; j++){
			if(isLineOfSight(particles[i].position,particleShadows[j].position)){
				edgeCost = round(distP2(particles[i].position,particleShadows[j].position)*10)/10;
				variables.push([variableCount,1,i,j,edgeCost,particles[i].position,particleShadows[j].position]); // 1 stands for link between particle & shadow
				variableCount++;
				objectiveFunction.push(edgeCost);
			}
			else{
				edgeCost = 100000;//inf length - dummy edge for initial BFS
				variables.push([variableCount,1,i,j,edgeCost,particles[i].position,particleShadows[j].position]); // 1 stands for link between particle & shadow
				variableCount++;
				objectiveFunction.push(edgeCost);
				
			}
		}
	}

	// updating wayPointsList
	wayPointsList = []; 
	for(var i = 0; i<obstacles.length; i++){
		for(var j = 0; j<obstacles[i].wayPoints.length; j++){
			wayPointsList.push(obstacles[i].wayPoints[j]);
		}
	}


	// edges between particles and wayPoints
	for(var i = 0; i<particles.length; i++){
		for(var j = 0; j<wayPointsList.length; j++){
			if(isLineOfSight(particles[i].position,wayPointsList[j])){
				edgeCost = round(distP2(particles[i].position,wayPointsList[j])*10)/10;
				variables.push([variableCount,2,i,j,edgeCost,particles[i].position,wayPointsList[j]]); // 2 stands for link between particle & wayPoint
				variableCount++;
				objectiveFunction.push(edgeCost);
			}
			else if(i==0){
				edgeCost = 100000;//inf length - dummy edge for initial BFS
				variables.push([variableCount,2,i,j,edgeCost,particles[i].position,wayPointsList[j]]); // 2 stands for link between particle & wayPoint
				variableCount++;
				objectiveFunction.push(edgeCost);
				
			}
		}
	}


	//edges between  wayPoints and wayPints
	for(var i = 0; i<(wayPointsList.length-1); i++){
		for(var j = (i+1); j<wayPointsList.length; j++){
			if(isLineOfSight(wayPointsList[i],wayPointsList[j])){
				edgeCost = round(distP2(wayPointsList[i],wayPointsList[j])*10)/10;
				variables.push([variableCount,3,i,j,edgeCost,wayPointsList[i],wayPointsList[j]]); // 4 stands for link between  wayPoint -wayPoint
				variableCount++;
				objectiveFunction.push(edgeCost);
			}
		}
	}


	//edges between  wayPoints and particleShadows
	for(var i = 0; i<wayPointsList.length; i++){
		for(var j = 0; j<particleShadows.length; j++){
			if(isLineOfSight(wayPointsList[i],particleShadows[j].position)){
				edgeCost = round(distP2(wayPointsList[i],particleShadows[j].position)*10)/10;
				variables.push([variableCount,4,i,j,edgeCost,wayPointsList[i],particleShadows[j].position]); // 4 stands for link between  wayPoint - shadow
				variableCount++;
				objectiveFunction.push(edgeCost);
			}
		}
	}

	//edges between  shadws and shadows -dummy
	for(var i = 0; i<(particleShadows.length-1); i++){
		var j = i+1
		edgeCost = 100000;
		variables.push([variableCount,5,i,j,edgeCost,particleShadows[i].position,particleShadows[j].position]); // 5 stands for link between  shadow - shadow
		variableCount++;
		objectiveFunction.push(edgeCost);

		
		
	}




	//print(variables);

	//// constructing the tableu
	var numOfEdges = variableCount;
	var numOfNodes = particles.length + wayPointsList.length + particleShadows.length ;
	var tableu = math.zeros(numOfNodes,numOfEdges);
	var startNode; 
	var endNode = 0;
	
	for(var i = 0; i<variableCount; i++){//column sweep
		if(variables[i][1]==1){// particles - shadows
			startNode = variables[i][2];
			endNode = particles.length + wayPointsList.length + variables[i][3];
			tableu._data[startNode][i] = 1;
			tableu._data[endNode][i] = -1;
		}else if(variables[i][1]==2){//particles - way points
			startNode = variables[i][2];
			endNode = particles.length + variables[i][3];
			tableu._data[startNode][i] = 1;
			tableu._data[endNode][i] = -1;
		}else if(variables[i][1]==3){//way points - way points
			startNode = particles.length + variables[i][2];
			endNode = particles.length + variables[i][3];
			tableu._data[startNode][i] = 1;
			tableu._data[endNode][i] = -1;
		}else if(variables[i][1]==4){//way points - shadows
			startNode = particles.length + variables[i][2];
			endNode = particles.length + wayPointsList.length + variables[i][3];
			tableu._data[startNode][i] = 1;
			tableu._data[endNode][i] = -1;
		}else if(variables[i][1]==5){//shadows - shadows
			startNode = particles.length + wayPointsList.length + variables[i][2];
			endNode = particles.length + wayPointsList.length + variables[i][3];
			tableu._data[startNode][i] = 1;
			tableu._data[endNode][i] = -1;
		}
		 

	}

	// b - vector
	var bVector = [];
	for(var i = 0; i<numOfNodes; i++){
		if (i < particles.length){
			bVector.push(1)
		}
		else if(i>=(particles.length+wayPointsList.length)){
			bVector.push(-1);
		}
		else{
			bVector.push(0);
		}
	}
	

	//print(tableu._data);
	//print(bVector);
	//print(objectiveFunction);

	var line1 = math.concat([0],objectiveFunction);
	//print(line1);

	tableu._data = math.transpose(tableu._data);
	
	tableu._data.splice(0,0,bVector);

	tableu._data = math.transpose(tableu._data);

	tableu._data.splice(0,0,line1);

	//print(tableu._data);
	
	minCostFlowVariables = variables;
	return tableu._data; //simplex tableu

		
}





function countingWithPhysicalAgents(){
	if(Math.abs(movingAverage)<15){
		countTillStaticMode++;
		if(countTillStaticMode>50){
			countTillStaticMode=51;
			print("Static Reached");
			if(staticMode==0){
				print("applying a purturbation");
				//purturbShadows();
				countTillStaticMode=0;
				staticMode=1;
				//trajectoryFollowButton.style("background-color", "grey");
				if(shadowsFollowingMode==1){
					shadowsFollowingMode=0;
					clearInterval(shadowsFollowingInterval);
					resetParticleHistory();
				}
			}
			else if(staticMode==1){
				print("freezed it");
				driveAgentsToTargets();   			
				
			}		
		}
	}
	else{
		countTillStaticMode=0;
		//trajectoryFollowButton.style("background-color", "grey");
		if(shadowsFollowingMode==1){
			shadowsFollowingMode=0;
			clearInterval(shadowsFollowingInterval);
			resetParticleHistory();
		}
		//print("Not Static")
		if(staticMode==2){
			print("break freeze")
			staticMode=0;

			shadowsFollowingMode = 0;
			//trajectoryFollowButton.style("background-color", "grey");
			
		
		}
		else if(staticMode==1){
			//print("just prtubed")
		}
		
	}
}

function testJustConvergence(){// when no boosting method is deployed

	var derivativeSumValues = [];
	var derivativeSumThresholds = [];

	for(var i = 0; i<particleShadows.length; i++){

		/*derivativeSumValues[i] = normP2(particleShadows[i].derivativeSumValue);*/

		derivativeSumValues[i] = particleShadows[i].derivativeAbsSquareSum;
		derivativeSumThresholds[i] = sq(descretizationLevel/2)/particleShadows[i].stepSizeSquareSum;
		////print(i+': '+derivativeSumValues[i]+' , '+derivativeSumThresholds[i]);
		particleShadows[i].derivativeAbsSquareSum = 0;//resetting
		particleShadows[i].stepSizeSquareSum = 0;//resetting


		if(variableStepSizeMode){
			derivativeSumThresholds[i] = particleShadows[i].lipschitzConstantK1*descretizationLevel/constantStepSize;
		}else{
			derivativeSumThresholds[i] = descretizationLevel/constantStepSize;
		}
		particleShadows[i].derivativeSumValue = new Point2(0,0);//resetting

		if(derivativeSumValues[i]<derivativeSumThresholds[i] && addObstacleMode==0 && isSimulationMode && !mouseIsPressed){
			particleShadows[i].localMinimaReached = true;
		}else{
			particleShadows[i].localMinimaReached = false;
		}
	}

}

function testLocalMinimaReached(){// centralized boosting

	//track change
	var newAgentPositions = [];

	var distanceIncrement = 0;
	var derivativeSumValue = 0;
	var derivativeSumValueThreshold = 0;
	

	for(var i = 0; i<particleShadows.length; i++){

		newAgentPositions[i] = particleShadows[i].position;

		
		/*derivativeSumValue = derivativeSumValue + normP2(particleShadows[i].derivativeSumValue);
		particleShadows[i].derivativeSumValue = new Point2(0,0);//resetting*/

		derivativeSumValue = derivativeSumValue + particleShadows[i].derivativeAbsSquareSum;
		derivativeSumValueThreshold = derivativeSumValueThreshold + sq(descretizationLevel/2)/particleShadows[i].stepSizeSquareSum;
		//////print(i+': '+derivativeSumValue+' , '+derivativeSumValueThreshold);
		particleShadows[i].derivativeAbsSquareSum = 0;//resetting
		particleShadows[i].stepSizeSquareSum = 0;//resetting

		//print(agentPositions[i]);
		//print((i==(particleShadows.length-1)) && (typeof agentPositions[i+1] !== "undefined"));
		if((i==(particleShadows.length-1)) && (typeof agentPositions[i+1] !== "undefined")){
			//print("agent has been removed");
			distanceIncrement = distanceIncrement + agentPositions[i+1].lengthP2();
		}
		else if(typeof agentPositions[i] !== "undefined"){
			//print("already in")
			distanceIncrement = distanceIncrement + distP2(newAgentPositions[i],agentPositions[i]);
		} 
		else{//new guy has appeared
			//print("New Agent Added");
			distanceIncrement = distanceIncrement + newAgentPositions[i].lengthP2();
		}

	}
	
	////print('Derivative Sum: ');
	////print(derivativeSumValue);

	agentPositions = newAgentPositions;
	//////if(distanceIncrement<10 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && boostingActivated==0){
	if((derivativeSumValue<derivativeSumValueThreshold || distanceIncrement<20) && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && boostingActivated==0){
		boostingActivated = 1; //for the first time boosting is turned ON
		
		optimalCoverageH1 = objectiveValue;
		optimalCoverageS1 = agentPositions;

		boostingIterationNumber = boostingIterationNumber + 1;
		consolePrint("Boosting activated; Iteration: "+boostingIterationNumber+"; Initial Objective : "+optimalCoverageH1);	

		// for VA - Boosting
        if(boostingMethod==7){// lock on to a target vertex anchor
        	var targetVertexAnchorFoundCount = 0;
        	for(var i = 0; i<particleShadows.length; i++){
            	var targetVertexAnchorFound = particleShadows[i].findTargetVertexAnchor();
            	if(targetVertexAnchorFound){targetVertexAnchorFoundCount++;}
            }
            if(targetVertexAnchorFoundCount==0){// none of the agents has a incentive to explore
                boostingActivated = 2; // switching off boosting immediately
                consolePrint("Boosting deactivated (No targets found !); switching to normal; Iteration: "+boostingIterationNumber);
            }
        }
        // end for VA - Boosting


	//////}else if(distanceIncrement<20 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && boostingActivated==1){
	}else if((derivativeSumValue<2*derivativeSumValueThreshold || distanceIncrement<20) && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && boostingActivated==1){
		boostingActivated = 2;//now switch back to normal procedure
		consolePrint("Boosting deactivated; switching to normal; Iteration: "+boostingIterationNumber);
	}
	//////else if(distanceIncrement<10 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && boostingActivated==2){
	else if((derivativeSumValue<derivativeSumValueThreshold || distanceIncrement<20) && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && boostingActivated==2){
		
		if (objectiveValue<=optimalCoverageH1){
			
			resetParticleShadowPositions(optimalCoverageS1);
			boostingActivated = 3;//stop boosting anymore
			
			// pause the simukation
			stopParticle();
			executionTime = Math.round(millis()-executionTime)/1000;
			// end pause

			if(displayPhysicalAgentsMode){
				consolePrint("Global optimality: ~"+objectiveValue+" reached in boosting iteration: "+boostingIterationNumber+"; Mobilizing Physical Agents...");
				driveAgentsToTargets();

			}else{
				consolePrint("Global optimality: ~"+objectiveValue+" reached in BIT: "+boostingIterationNumber+" after "+executionTime+" s");
			}




		}else{
			consolePrint("Boosting resetted; Improvement = "+(objectiveValue-optimalCoverageH1)+"; Iteration: "+boostingIterationNumber);
			optimalCoverageH1 = objectiveValue;
			optimalCoverageS1 = agentPositions;
			boostingActivated = 0;//start over
		}
		
	}
	else if(boostingActivated==3 && distanceIncrement>100){
		boostingActivated = 0; //reactivate boosting
		boostingIterationNumber = 0;

		for(var i = 0; i<particleShadows.length; i++){//resetting dimsteps
			particleShadows[i].diminishingStepSizeModeInitiated = false;
		}
		consolePrint("Boosting reactivated");
	}
	else{

	}
	//print(distanceIncrement);
}

function driveAgentsToTargets(){
	staticMode=2;
	if(shadowsFollowingMode==0){
		shadowsFollowingMode=1;
		
		//optimization - assignment problem
		if(obstacles.length==0){// if no obstacles, then it would be just assignment
			
			var costMatrix = formCostMatrix();
			//print(costMatrix);
			var assignments = solveAssignmentProblem(costMatrix);
			//print(assignments);


			var costSaved = 0;
			for(var i=0; i<assignments.length; i++){
				costSaved = costSaved+( costMatrix[i][i] - costMatrix[i][assignments[i][1]] );
			}
			print("Saved Distance: "+costSaved);
			
			for(var i = 0; i<particleShadows.length; i++){
				append(particles[i].wayPoints,particleShadows[assignments[i][1]].position);
				particles[i].generateReferencePointTrajectory();
			}

		}else{// when obstacles are there, need to use 

			//construct graph
			var simplexTableu = formSimplexTableu();

			//solve graph
			minCostFlowSolution = solveSimplexTableu(simplexTableu);

			if(minCostFlowSolution.length>0){
				consolePrint("Optimal Solution Found !!! ");
				minCostFlowSolutionTemp = minCostFlowSolution;
				for(var i = 0; i<particles.length; i++){
																				
					var travelPoints = interpretSolution(minCostFlowSolutionTemp,i)
					for(var j = 0; j<travelPoints.length; j++){
						append(particles[i].wayPoints,travelPoints[j]);
					}
					particles[i].generateReferencePointTrajectory();
				}

			}else{
				consolePrint("Optimal Solution Not Found !!!");
			}


		}				
			    				
		shadowsFollowingInterval = setInterval(followShadows,100);
		//trajectoryFollowButton.style("background-color", "red");
	}		    			
	
}

function strip(html)
{
   var tmp = document.createElement("DIV");
   tmp.innerHTML = html;
   return tmp.textContent || tmp.innerText || "";
}
function convertHtmlToText(inputText) {
    //var inputText = document.getElementById("input").value;
    var returnText = "" + inputText;

    //-- remove BR tags and replace them with line break
    returnText=returnText.replace(/<br>/gi, "\n");
    returnText=returnText.replace(/<br\s\/>/gi, "\n");
    returnText=returnText.replace(/<br\/>/gi, "\n");

    //-- remove P and A tags but preserve what's inside of them
    returnText=returnText.replace(/<p.*>/gi, "\n");
    returnText=returnText.replace(/<a.*href="(.*?)".*>(.*?)<\/a>/gi, " $2 ($1)");

    //-- remove all inside SCRIPT and STYLE tags
    returnText=returnText.replace(/<script.*>[\w\W]{1,}(.*?)[\w\W]{1,}<\/script>/gi, "");
    returnText=returnText.replace(/<style.*>[\w\W]{1,}(.*?)[\w\W]{1,}<\/style>/gi, "");
    //-- remove all else
    returnText=returnText.replace(/<(?:.|\s)*?>/g, "");

    //-- get rid of more than 2 multiple line breaks:
    returnText=returnText.replace(/(?:(?:\r\n|\r|\n)\s*){2,}/gim, "\n\n");

    //-- get rid of more than 2 spaces:
    returnText = returnText.replace(/ +(?= )/g,'');

    //-- get rid of html-encoded characters:
    returnText=returnText.replace(/&nbsp;/gi," ");
    returnText=returnText.replace(/&amp;/gi,"&");
    returnText=returnText.replace(/&quot;/gi,'"');
    returnText=returnText.replace(/&lt;/gi,'<');
    returnText=returnText.replace(/&gt;/gi,'>');
    return returnText;
    //-- return
    //document.getElementById("output").value = returnText;
}

function calculateGlobalObjectiveValuesWRTs_N(){
	var N = particleShadows.length-1;
	var stepSize = 5;
	var startPos = particleShadows[N].position;

	var rowNum = 0;
	
	var HValues = [];
	var HValuesX = [];
	var HValuesY = [];
	var firstRow = true;

	for(var y = 0; y < width; y = y + stepSize){
		HValues.push([]);
		HValuesY.push(y);

		for(var x = 0; x < height; x = x + stepSize){
			interestedPoint = new Point2(x,y);
			if(getEventDensity(interestedPoint)>0){
				particleShadows[N].position = new Point2(x,y);
				HValues[rowNum].push(globalObjective());
			}else{
				HValues[rowNum].push(NaN);
			}
			if(firstRow){
				HValuesX.push(x);
			}
		}
		firstRow = false;
		rowNum ++;
	}

	particleShadows[N].position = startPos;

	globalObjectiveValuesWRTs_N = HValues;
	globalObjectiveValuesWRTs_NX = HValuesX;
	globalObjectiveValuesWRTs_NY = HValuesY;
	print(globalObjectiveValuesWRTs_N);
}

function moveGraduallyandObserve(agentID,startPoint,endPoint){
	particleShadows[agentID].position = startPoint;
	
	descretizationLevel = 0.5;
	stepSizeValue = 0.1;
	numOfSteps = distP2(startPoint,endPoint)/stepSizeValue;
	normalDirection = normalizeP2(minusP2(endPoint,startPoint));
	stepVector = productP2(normalDirection,stepSizeValue);
	array = [];
	fullArray = [];

	for(var i = 0; i < numOfSteps; i++){
		
		var derivative = particleShadows[agentID].getDerivatives();
		array = [];
		array.push(particleShadows[agentID].position.x);
		array.push(particleShadows[agentID].position.y);
		array.push(derivative[0]);
		array.push(derivative[1]);

		fullArray.push(array);
		particleShadows[agentID].position = plusP2(particleShadows[agentID].position,stepVector);
		//particleShadows[agentID].showShadow();
	
	}

	print('End! Saving and downloading data');

	textToWrite = "a = [";
	textToWrite = textToWrite + fullArray.toString();
	/*for(var i=0; i<fullArray.length; i++){
    	textToWrite = textToWrite + fullArray[i].toString()+";\r\n";
    }*/
    textToWrite = textToWrite + "]";
	// var textToWrite = "["+array.toString()+"]";
    
    ////var textToWrite = convertHtmlToText(document.getElementById("consoleText").innerHTML);
    var textFileAsBlob = new Blob([textToWrite], {type:'text/plain'});
    var fileNameToSaveAs = "SimulationData";//document.getElementById("inputFileNameToSaveAs").value;
    var downloadLink = document.createElement("a");
    
    downloadLink.download = fileNameToSaveAs;
    downloadLink.innerHTML = "Download File";
    if (window.URL != null)
    {
        // Chrome allows the link to be clicked
        // without actsually adding it to the DOM.
        downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
    }
    else
    {
        // Firefox requires the link to be added to the DOM
        // before it can be clicked.
        downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
        downloadLink.onclick = destroyClickedElement;
        downloadLink.style.display = "none";
        document.body.appendChild(downloadLink);
    }

    downloadLink.click();
}




function saveTextAsFile(){
    ////var textToWrite = textToSave;//document.getElementById("inputTextToSave").value;
    ////var textToWrite = strip(document.getElementById("consoleText").innerHTML);
    if(selectedDataToSave==0){
    	var textToWrite = convertHtmlToText(document.getElementById("consoleText").innerHTML);
    }else{
	    var textToWrite = "";
	    for(var i=0; i<particleShadows.length; i++){
	    	textToWrite = textToWrite+"a"+i.toString()+"=["+TValuesArray[i].toString()+"]\r\n";
	    }
	}
    
    var textFileAsBlob = new Blob([textToWrite], {type:'text/plain'});
    var fileNameToSaveAs = "SimulationData";//document.getElementById("inputFileNameToSaveAs").value;
    var downloadLink = document.createElement("a");
    
    downloadLink.download = fileNameToSaveAs;
    downloadLink.innerHTML = "Download File";
    if (window.URL != null)
    {
        // Chrome allows the link to be clicked
        // without actsually adding it to the DOM.
        downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
    }
    else
    {
        // Firefox requires the link to be added to the DOM
        // before it can be clicked.
        downloadLink.href = window.URL.createObjectURL(textFileAsBlob);
        downloadLink.onclick = destroyClickedElement;
        downloadLink.style.display = "none";
        document.body.appendChild(downloadLink);
    }

    downloadLink.click();
}

function plotButtonClicked(){
	localStorage.setItem('key1', textToSave);
	calculateGlobalObjectiveValuesWRTs_N();
	print("plotting");
	
}


function testAgentLocalMinimaReached(){// decentralized boosting !!!! old code -moved this to particle.js later

	//track change
	//print("agent wise");
	var newAgentPositions = [];
	var systemPurturbed = false;
	var distanceIncrements = [];

	var derivativeSumValues = [];
	var derivativeSumThresholds = [];

	for(var i = 0; i<particleShadows.length; i++){

		newAgentPositions[i] = particleShadows[i].position;

		// Using total distance via gradient sum
		/*derivativeSumValues[i] = normP2(particleShadows[i].derivativeSumValue);
		if(variableStepSizeMode){
			derivativeSumThresholds[i] = particleShadows[i].lipschitzConstantK1*descretizationLevel/constantStepSize
		}else{
			derivativeSumThresholds[i] = descretizationLevel/constantStepSize;
		}
		particleShadows[i].derivativeSumValue = new Point2(0,0);//resetting*/

		// Using abs square sum via gradient sum
		derivativeSumValues[i] = particleShadows[i].derivativeAbsSquareSum;
		derivativeSumThresholds[i] = sq(descretizationLevel/2)/particleShadows[i].stepSizeSquareSum;
		//////print(i+' cleared: '+derivativeSumValues[i]+' , '+derivativeSumThresholds[i]);
		particleShadows[i].derivativeAbsSquareSum = 0;//resetting
		particleShadows[i].stepSizeSquareSum = 0;//resetting


		if((i==(particleShadows.length-1)) && (typeof agentPositions[i+1] !== "undefined")){
			//print("agent has been removed");
			systemPurturbed = true;
			distanceIncrements[i] = distP2(newAgentPositions[i],agentPositions[i]);
		}
		else if(typeof agentPositions[i] !== "undefined"){
			//print("already in")
			distanceIncrements[i] = distP2(newAgentPositions[i],agentPositions[i]);
		} 
		else{//new guy has appeared
			//print("New Agent Added");
			systemPurturbed = true; 
			distanceIncrements[i] = newAgentPositions[i].lengthP2();
		}
		//particleShadows[i].coverageLevel = particleShadows[i].objectiveFunction();
	}
	
	agentPositions = newAgentPositions;
	var optimalityReachedShadowCount = 0;
	//distanceIncrements
	//print(distanceIncrements);
	////print('Derivative Sum:');
	////print(derivativeSumValues);

	for(var i = 0; i<particleShadows.length; i++){// this means we can decentralize it easily

		// check for neighborhood changes
		var continueIteration = true;
		if(addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==1){
			//print("here")
			//var neighborsNew = particleShadows[i].getNeighbors();
			var neighborsNew = particleShadows[i].getEffectiveNeighbors();
			var neighborsOld = particleShadows[i].neighborsOld;
			var combinedCoverageNew = particleShadows[i].getLocalCombinedCoverageLevel();
			var combinedCoverageOld = particleShadows[i].combinedCoverageOld;

			if(isEqualArrays(neighborsOld,neighborsNew)){
				// all is fine; so far neighborhood has not changed
				particleShadows[i].combinedCoverageOld = combinedCoverageNew; 
				//if(particleShadows[i].optimalCoverageH1==0){print("here0");}
			}else{// neighbor set has been changed
				// has coverage improved during previous session? If yes, carry on boosting stage; else
				continueIteration = false;
				if(combinedCoverageOld > particleShadows[i].optimalCoverageH1){// boosting has improved the coverage untill last point
					// need to carry on the boosting stage; make appropriate changes
					particleShadows[i].optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
					particleShadows[i].optimalCoverageS1 = agentPositions[i]; //new staring point
					particleShadows[i].neighborsOld = neighborsNew; // new neighbor set
					particleShadows[i].combinedCoverageOld = combinedCoverageNew;
					consolePrint("Agent "+(i+1)+"'s neighborhood changed while boosting; Coverage improved from boosting; Boosting continues");
					//if(particleShadows[i].optimalCoverageH1==0){print("here1");}
				}else{
					particleShadows[i].optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
					particleShadows[i].optimalCoverageS1 = agentPositions[i]; //new staring point
					particleShadows[i].neighborsOld = neighborsNew; // new neighbor set
					particleShadows[i].combinedCoverageOld = combinedCoverageNew;


					particleShadows[i].isBoostingActivated = 2;//stop boosting temporary
					consolePrint("Agent "+(i+1)+"'s neighborhood changed while boosting; No coverage improvement from boosting; Boosting temporarily halted; Iteration: "+boostingIterationNumber);
					//if(particleShadows[i].optimalCoverageH1==0){print("here1.5");}
				}


			}
			
		}
		else if(addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated>=2 && particleShadows[i].isBoostingActivated<=8){
			//print("here")
			//var neighborsNew = particleShadows[i].getNeighbors();
			var neighborsNew = particleShadows[i].getEffectiveNeighbors();
			var neighborsOld = particleShadows[i].neighborsOld;
			var combinedCoverageNew = particleShadows[i].getLocalCombinedCoverageLevel();
			var combinedCoverageOld = particleShadows[i].combinedCoverageOld;

			if(isEqualArrays(neighborsOld,neighborsNew)){
				// all is fine; so far neighborhood has not changed
				particleShadows[i].combinedCoverageOld = combinedCoverageNew; 
			}else{// neighbor set has been changed - reactivate boosting
				continueIteration = false;
				if(combinedCoverageOld > particleShadows[i].optimalCoverageH1){ // waiting has improved the coverage during previous session (stay same)
					particleShadows[i].optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
					particleShadows[i].optimalCoverageS1 = agentPositions[i]; //new staring point
					particleShadows[i].neighborsOld = neighborsNew; // new neighbor set
					particleShadows[i].combinedCoverageOld = combinedCoverageNew;

					//particleShadows[i].isBoostingActivated = 0;//reset boosting
					consolePrint("Agent "+(i+1)+"'s neighborhood changed while in normal mode; Coverage improved from normal mode; Normal mode continues");
				}else{// waiting has not incresed coverage
					particleShadows[i].optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
					particleShadows[i].optimalCoverageS1 = agentPositions[i]; //new staring point
					particleShadows[i].neighborsOld = neighborsNew; // new neighbor set
					particleShadows[i].combinedCoverageOld = combinedCoverageNew;
					
					consolePrint("Agent "+(i+1)+"'s neighborhood changed while in normal mode; No coverage improvement from normal mode; Normal mode continues");
				}
			}
		
		}
		// end check for neighborhood change



		//////if(continueIteration && distanceIncrements[i]<4 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==0 && millis()>3000){
		if(continueIteration && derivativeSumValues[i]<derivativeSumThresholds[i] && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==0){
			
			particleShadows[i].isBoostingActivated = 1; //for the first time boosting is turned ON
			
			if(particleShadows[i].boostingIterationNumber==0){
				particleShadows[i].optimalCoverageH1 = particleShadows[i].getLocalCombinedCoverageLevel(); // this is local info
				particleShadows[i].optimalCoverageS1 = agentPositions[i];
				//if(particleShadows[i].optimalCoverageH1==0){print("here2");}
			}
			// keep track of neighbors and their positions
			
			//particleShadows[i].neighborsOld = particleShadows[i].getNeighbors(); // list of neighbors when started
			particleShadows[i].neighborsOld = particleShadows[i].getEffectiveNeighbors(); // list of neighbors when started
			particleShadows[i].combinedCoverageOld = particleShadows[i].optimalCoverageH1; // this will be updated at each interval
			
			particleShadows[i].boostingIterationNumber++;

			consolePrint("Agent "+(i+1)+"'s boosting activated; Iteration: "+particleShadows[i].boostingIterationNumber);

		//////}else if(continueIteration && distanceIncrements[i]<4 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==1){
		}else if(continueIteration && derivativeSumValues[i]<2*derivativeSumThresholds[i] && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==1){
			
			particleShadows[i].isBoostingActivated = 2;//now switch back to normal procedure
			
			consolePrint("Agent "+(i+1)+"'s boosting deactivated; switching to normal; Iteration: "+particleShadows[i].boostingIterationNumber);
			
		}
		//////else if(continueIteration && distanceIncrements[i]<5 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==2){
		else if(continueIteration && derivativeSumValues[i]<derivativeSumThresholds[i] && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==2){
			
			var improvement = particleShadows[i].getLocalCombinedCoverageLevel()-particleShadows[i].optimalCoverageH1;
			// if(improvement>10000){
			// 	print(i,particleShadows[i].getLocalCombinedCoverageLevel(),particleShadows[i].combinedCoverageOld,particleShadows[i].optimalCoverageH1);
			// 	print(i,particleShadows[i].getNeighbors(),particleShadows[i].neighborsOld);
			// }
			if (improvement<=0){//objectiveValue<=optimalCoverageH1
				
				particleShadows[i].isBoostingActivated = 3;//stop boosting anymore start waiting period
				
				if(displayPhysicalAgentsMode){
					consolePrint("Agent "+(i+1)+"'s initial boosting/normal session finished; No improvement; Iteration: "+particleShadows[i].boostingIterationNumber+"; Mobilizing Physical Agents...");
					driveAgentsToTargets();

				}else{
					consolePrint("Agent "+(i+1)+"'s initial boosting/normal session finished; No improvement; Iteration: "+particleShadows[i].boostingIterationNumber);
					
				}

			}else{
								
				consolePrint("Agent "+(i+1)+"'s initial boosting/normal session finished; Coverage improvement (of "+round(improvement)+") achieved; boosting resetted; Iteration: "+particleShadows[i].boostingIterationNumber);
				
				particleShadows[i].optimalCoverageH1 = particleShadows[i].getLocalCombinedCoverageLevel();
				particleShadows[i].optimalCoverageS1 = agentPositions[i];
				particleShadows[i].isBoostingActivated = 0;//start over

			}
			
		}		
		//////else if(continueIteration && distanceIncrements[i]<5 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated>=3 && particleShadows[i].isBoostingActivated<8){
		else if(continueIteration && derivativeSumValues[i]<derivativeSumThresholds[i] && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated>=3 && particleShadows[i].isBoostingActivated<8){
			// waiting period 
			particleShadows[i].isBoostingActivated = particleShadows[i].isBoostingActivated + 1;

		}
		//////else if(continueIteration && distanceIncrements[i]<5 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==8){
		else if(continueIteration && derivativeSumValues[i]<derivativeSumThresholds[i] && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && particleShadows[i].isBoostingActivated==8){

			var improvement = particleShadows[i].getLocalCombinedCoverageLevel()-particleShadows[i].optimalCoverageH1;
			if (improvement<=0){//objectiveValue<=optimalCoverageH1
				
				// no resetting 
				//resetParticleShadowPositions(agentPositions);
				////particleShadows[i].x = particleShadows[i].optimalCoverageS1.x;
				////particleShadows[i].y = particleShadows[i].optimalCoverageS1.y;
				////particleShadows[i].position = particleShadows[i].optimalCoverageS1;

				particleShadows[i].isBoostingActivated = 9;//stop boosting anymore
				if(displayPhysicalAgentsMode){
					consolePrint("Agent "+(i+1)+"'s boosting sequence finished; No improvement achieved from boosting; Iteration: "+particleShadows[i].boostingIterationNumber+"; Mobilizing Physical Agents...");
					driveAgentsToTargets();

				}else{
					
					consolePrint("Agent "+(i+1)+"'s boosting sequence finished; No improvement achieved from boosting; Iteration: "+particleShadows[i].boostingIterationNumber+"; Final Objective : "+round(particleShadows[i].getLocalCombinedCoverageLevel()*100)/100);
					
				}

			}else{
								
				consolePrint("Agent "+(i+1)+"'s boosting sequence resetted; Improvement (of "+round(improvement)+") achieved; Iteration: "+boostingIterationNumber);
				
				particleShadows[i].optimalCoverageH1 = particleShadows[i].getLocalCombinedCoverageLevel();
				particleShadows[i].optimalCoverageS1 = agentPositions[i];
				particleShadows[i].isBoostingActivated = 0;//start over

			}

		}
		else if(continueIteration && (distanceIncrements[i]>10 || systemPurturbed) && particleShadows[i].isBoostingActivated==9){
		//////else if(continueIteration && (derivativeSumValues[i]>10 || systemPurturbed) && particleShadows[i].isBoostingActivated==9){
			
			particleShadows[i].isBoostingActivated = 0; //reactivate boosting
			particleShadows[i].boostingIterationNumber = 0;
			
			particleShadows[i].diminishingStepSizeModeInitiated = false;
			consolePrint("Agent "+(i+1)+"'s boosting reactivated due to motion");
		

		}
		else if(continueIteration && particleShadows[i].isBoostingActivated==9){
			optimalityReachedShadowCount++
		}else{

		}

	}

	if(systemPurturbed){
		systemPurturbed = false;
	}
	if(optimalityReachedShadowCount==particleShadows.length){
		consolePrint("Global optimality: ~"+objectiveValue+" reached");
	}


}


function updateAndStoreDataForPlotting(){

	objectiveValueArray.splice(0,1);    
    objectiveValueArray.push(objectiveValueNew);
    
    objectiveValueIncrementArray.splice(0,1);    
    objectiveValueIncrementArray.push(objectiveValueNew-objectiveValue);
    
    iterationNumberArray.splice(0,1);
    iterationNumberArray.push(iterationNumber);

    //globalObjectiveValuesWRTs_N = calculateGlobalObjectiveValuesWRTs_N();

    var trace1 = {x:iterationNumberArray, y: objectiveValueArray,type: 'scatter'};
    var trace2 = {x:iterationNumberArray, y: objectiveValueIncrementArray,type: 'scatter'};
    
    var trace3 = {
    	z: globalObjectiveValuesWRTs_N,
    	x: globalObjectiveValuesWRTs_NX,
  		y: globalObjectiveValuesWRTs_NY,
		type: 'surface',
		  contours: {
		    z: {
		      show:true,
		      usecolormap: true,
		      highlightcolor:"#42f462",
		      project:{z: true}
		    }
		  }
	};

    var data1 = [trace1];
    var data2 = [trace2];
    var data3 = [trace3];
    var data1s = JSON.stringify(data1);
    var data2s = JSON.stringify(data2);
    var data3s = JSON.stringify(data3);
    localStorage.setItem('key1', data1s);
    localStorage.setItem('key2', data2s);
    localStorage.setItem('key4', data3s);
    

    //plotting derivatives and beta values
    if(derivativeValuesArray.length==200){

        derivativeValuesArray.splice(0,1);
        derivativeValuesArray.push(derivativeValues);
    	
        betaValuesArray.splice(0,1);
        betaValuesArray.push(betaValues);

        QValuesArray.splice(0,1);
        QValuesArray.push(QValues);
    
    }else{
        derivativeValuesArray.push(derivativeValues);
        betaValuesArray.push(betaValues);
        QValuesArray.push(QValues);
    }

    


    // create array of traces corresponding to agent derivatives
    localStorage.setItem('key3', particleShadows.length.toString());// to coomunicate the number of the agents
    for(var i=0; i<particleShadows.length; i++){

    	for(j=0;j<200;j++){
    		// derivatives
    		if(typeof derivativeValuesArray[j] == "undefined"){
    			derivativeValuesArray[j] = [];
    		}
    		if(typeof derivativeValuesArray[j][i] == "undefined"){
    			derivativeValuesArrayTemp[j] = 0;	
    		}else{
    			derivativeValuesArrayTemp[j] = derivativeValuesArray[j][i];
    		}

    		// beta
    		if(typeof betaValuesArray[j] == "undefined"){
    			betaValuesArray[j] = [];
    		}
    		if(typeof betaValuesArray[j][i] == "undefined"){
    			betaValuesArrayTemp[j] = 0;	
    		}else{
    			betaValuesArrayTemp[j] = betaValuesArray[j][i];
    		}


    		// Q
    		if(typeof QValuesArray[j] == "undefined"){
    			QValuesArray[j] = [];
    		}
    		if(typeof QValuesArray[j][i] == "undefined"){
    			QValuesArrayTemp[j] = 0;	
    		}else{
    			QValuesArrayTemp[j] = QValuesArray[j][i];
    		}
			
		}

		// derivative
    	var trace = {x:iterationNumberArray, y: derivativeValuesArrayTemp,type: 'scatter'};
    	var data = [trace];
    	var datas = JSON.stringify(data);
    	localStorage.setItem('keyD'+(1+i), datas);

    	// beta 
    	var trace = {x:iterationNumberArray, y: betaValuesArrayTemp,type: 'scatter'};
    	var data = [trace];
    	var datas = JSON.stringify(data);
    	localStorage.setItem('keyB'+(1+i), datas);

    	// Q 
    	var trace = {x:iterationNumberArray, y: QValuesArrayTemp,type: 'scatter'};
    	var data = [trace];
    	var datas = JSON.stringify(data);
    	localStorage.setItem('keyQ'+(1+i), datas);
    }

   

 
    

}



function startRoutine1(){

	for(var i = 0; i < 5; i++){
		addAgent();
	}

	document.getElementById("sensingRange").value = "100";
	sensingRangeChanged("100");

	document.getElementById("sensingDecay").value = "8";
	sensingDecayChanged("8");
	
	for(var i = 0; i < 4; i++){
		addAgent();
	}

	generateCandidatesBtnFcn();
	solveCentralizedGreedyBtnFcn();
	setTimeout(deployAgentsBtnFcn, 1000);

	//deployAgentsBtnFcn();

}


