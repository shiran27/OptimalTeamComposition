var particles = [];

var particleShadows = [];

var sensitivity=0.1;

var linearSpeed=0;

var senRange;

var descretizationLevel;

var coverageLevelDisplay;

var objectiveValue=0;

var objectiveValueNew;

var objectiveValueArray = Array.apply(null, Array(200)).map(Number.prototype.valueOf,0);

var objectiveValueIncrementArray = Array.apply(null, Array(200)).map(Number.prototype.valueOf,0);

var globalObjectiveValuesWRTs_N = [
    [8.83,8.89,8.81,8.87,8.9,8.87],
    [8.89,8.94,8.85,8.94,8.96,8.92],
    [8.84,8.9,8.82,8.92,8.93,8.91],
    [8.79,8.85,8.79,8.9,8.94,8.92],
    [8.79,8.88,8.81,8.9,8.95,8.92],
    [8.8,8.82,8.78,8.91,8.94,8.92],
    [8.75,8.78,8.77,8.91,8.95,8.92],
    [8.8,8.8,8.77,8.91,8.95,8.94],
    [8.74,8.81,8.76,8.93,8.98,8.99],
    [8.89,8.99,8.92,9.1,9.13,9.11],
    [8.97,8.97,8.91,9.09,9.11,9.11],
    [9.04,9.08,9.05,9.25,9.28,9.27],
    [9,9.01,9,9.2,9.23,9.2],
    [8.99,8.99,8.98,9.18,9.2,9.19],
    [8.93,8.97,8.97,9.18,9.2,9.18]
];

globalObjectiveValuesWRTs_NY = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14];
globalObjectiveValuesWRTs_NX = [10,11,12,13,14,15];

var derivativeValuesArrayTemp = Array.apply(null, Array(200)).map(Number.prototype.valueOf,0);
var derivativeValuesArray = [];
var derivativeValues = []; // to hold the derivative values of each agent temporarily

var betaValuesArrayTemp = Array.apply(null, Array(200)).map(Number.prototype.valueOf,0);
var betaValuesArray = [];
var betaValues = []; // to hold the derivative values of each agent temporarily


var QValuesArrayTemp = Array.apply(null, Array(200)).map(Number.prototype.valueOf,0);
var QValuesArray = [];
var QValues = []; // to hold the derivative values of each agent temporarily



var iterationNumberArray = Array.apply(null, {length: 200}).map(Number.call, Number);

var iterationNumber=0;

var movingAverage = 0; //starting mean

var movingAverageWindowSize = 50; 



var dwellTimeSteps = 50;

var particleDraggingMode = false;

var particleDragging;


var dwellMode = false;

var shadowsFollowingMode = false;//maybe same as isSimulationMode

var staticMode = 0;

var countTillStaticMode = 0;

var pausedMode = false;

var plotLayout = {
		title: 'Coverage Cost Increment Vs Iteration Number', 
		autosize: true,
	    width: 500,
	    height: 300,
	    xaxis: {
			title: 'Iteration Number', 
			showline: true,
			showgrid: true, 
			zeroline: true,
		}, 
		yaxis: {
			title: 'Cost Increment', 
		    showline: true,
		    showgrid: true,
		    zeroline: true,
		}
}


var cnv;

var boxDis;

var dropdown;

var hStep=0.5;//stepsize for updating the position

var ItterNum=1;//itteration numbers for updating the position

var obstacles = [];

var addObstacleMode = 0;

var isSimulationMode = false;


var obstacleColor;

//for line searching
var searchResolution = 10;

var sensingDecayFactor; // 0.012

var simulationTime = 0;

var timeStep = 0.1;

var timeStepSqHf = timeStep*timeStep/2;

var trajectoryFollowInterval;

var trajectoryFollowMode = 0;

var pickWayPointsMode = 0;



var pixelMap = [];
var numberOfObstaclesOld = 0;

var width;
var height;
var numberOfAgentsDisplay;
var displayPhysicalAgentsMode = true;

var boostingMethod = 0;

////var testLocalMinimaReachedInterval;
var testLocalMinimaReachedInterval = 0;

var agentPositions = []; // this is not good to asses the convergence especially when step size is varying

var localMinimaReached = false;
var boostingActivated = 0;
var boostingIterationNumber = 0;

var optimalCoverageH1 = 0;
var optimalCoverageS1 = [];
var boostingParameterK;
var boostingParameterY;

//var optimalCoverageH2 = 0;
var wayPointsList = [];
var minCostFlowVariables = [];
var minCostFlowSolution = -1;
var minCostFlowSolutionTemp = -1;

var textToSave = "";
var saveDataMode;

var decentralizedBoostingMethod;

//////var testAgentLocalMinimaReachedInterval;
var testAgentLocalMinimaReachedInterval = 0; //number of simulation iteratipons 

var boundaryObstacle;

// PSO related
var solutionOfPSO = []; //pointArray corresponding to the solution
var solutionOfPSOTrace = [];
var simulatePSOInterval;
var simulatePSOIntervalStep=0;
var pointsToBePrinted = [];


// debug mode

var isDebugMode = false;
var variableStepSizeMode;
var variableStepSizeModeSuppressed;
var simulationStartIterationNumber = 0; 
var negativeStepsAllowedMode = true;

var diminishingStepSizeMode;
var diminishingStepSizeK0;
var diminishingStepSizer;

var lipschitzConstantK1 = 0.2;
var lipschitzConstantK1Hat = 1;

var constantStepSize = 1; //this is important
 
var plotDataMode = true; 

// data storing
var DeltaValuesMatrix = [];
var Q_iSum =[];
var TValuesArray = [];

var startButtonClicked = false;
var startButtonClickedIteration = 0;
var executionTime = 0;


//////var updateLocalLipschitzConstantsInterval;
var updateLocalLipschitzConstantsInterval = 0;
var autoTuneLipschitzMode;

var stateUpdateIterationNumber = 0;

var testJustConvergenceInterval = 0;

var selectedDataToSave = 0;

// submodularity related
var submodularityMode = 0;
var submodularityCandidates = [];
var submodularitySolution = [];
var savedParticles = []; // to keep particle shadows stored
var savedParameters = [];

// paper 3
var weightPGD = -15000;
var executionTimeGreedy = 0;
var executionTimePGD = 0;
var particleSelectDropdownUsed = 0;
var agentCostRatio = 1;


function setup() {

	pixelDensity(1);
	
	//Basic configurations
	
	obstacleColor = color(46, 139, 87);
	
	const canvasHolder = select('#canvasHolder');
    width  = canvasHolder.width;
    height = canvasHolder.height;
    cnv = createCanvas(width, height);
    cnv.parent('canvasHolder');
    //console.log(canvasHolder);
    print(width + ', ' + height);
    

    var boundaryCoordinates = [0,0,width,0,width,height,0,height];
   	boundaryObstacle = new Obstacle();
	boundaryObstacle.x = [0,width,width,0];
	boundaryObstacle.y = [0,0,height,height];
	boundaryObstacle.calculateReflexVertices();



	var option1 = document.createElement("option");
    option1.text = "All";
    document.getElementById("particleSelectDropdown").add(option1);
    document.getElementById("particleSelectDropdown").selectedIndex = 0;

    addAgent();
    /*particles[0] = new Particle(15,15);
    
    particleShadows[0] = new Particle(15,15);*/

    frameRate(60);
    
    
    obstacleSetupDropdownEvent();
    // decentralizedBoostingMethodDropdownEvent();
    // boostingMethodChanged();
    // boostingParameterChanged();
    boostingMethod = 0;
    selectDataToSaveDropdownEvent();
    //calc reflex vertices of obstacles
    
    for(var i = 0; i < obstacles.length; i++){
    	obstacles[i].calculateReflexVertices();
    }
    
	numberOfObstaclesOld = obstacles.length;    
    variableStepSizeModeChanged();
    diminishingStepSizeModeChanged();
    
    ////autoTuneLipschitzModeChanged();
    //////clearInterval(updateLocalLipschitzConstantsInterval);
    updateLocalLipschitzConstantsInterval = 0;

    consolePrint("Interface Loading Finished.");
    
    
}



function initiateDebugging(){
    if (isDebugMode) {
        isDebugMode = false;
        start();
    }
    else{
        stopParticle();
        isDebugMode = true;
    }
}

//step size dropdown function

function mySelectEvent() {

    var selected = document.getElementById("stepSizeMultiplier").value;
    if (selected === '1') {
        hStep=1;
        ItterNum=1;
        print("1");
    }

    if (selected === '2') {
        hStep=0.1;
        ItterNum=10;
    }

    if (selected === '3') {
        hStep=0.01;
        ItterNum=100;
    }

}


//start button function
function initiateStart(){
      consolePrint("Initializing – Please Wait"); 
      document.getElementById("alertBox").style.display = 'block'; 
      startButtonClickedIteration = iterationNumber;
      startButtonClicked = true;
}



function start(){

    /////if(variableStepSizeMode){
        //consolePrint("Initializing – Please Wait"); 
        //window.alert("Initializing – Please Wait");

        ////tuneLipschitzConstants();
        if(autoTuneLipschitzMode){
            tuneLipschitzConstantsLocally();
            //////clearInterval(updateLocalLipschitzConstantsInterval);
            updateLocalLipschitzConstantsInterval = 0;
            //////updateLocalLipschitzConstantsInterval = setInterval(tuneLipschitzConstantsLocally,5000);
            updateLocalLipschitzConstantsInterval = 100;
        }else{
            //////clearInterval(updateLocalLipschitzConstantsInterval);
            updateLocalLipschitzConstantsInterval = 0;
            tuneLipschitzConstants();
        }
        
        simulationStartIterationNumber = stateUpdateIterationNumber;
	////////}

    isSimulationMode = true;
	linearSpeed = document.getElementById("stepSize").value;
    if(pausedMode==1){
    	shadowsFollowingMode=1;
    	shadowsFollowingInterval = setInterval(followShadows,100);
    	pausedMode = 0; 

    }

    consolePrint("Simulation Started."); 
	

    document.getElementById("alertBox").style.display = 'none'; 
    startButtonClicked  = false;
    executionTime = millis();
}


//end start button functions

// reset button function
function reset(){

    linearSpeed = 0;
    isSimulationMode = false;
 
    for (var i = 0; i < particles.length; i++){//resetting positions 
        
    	//resetting particles
    	////particles[i].position.x=10*i+15;
        ////particles[i].y=10*i+15;
        ////particles[i].position = new Point2(particles[i].x,particles[i].y);
        
        //resetting particle shadows
        particleShadows[i].position = new Point2(10*i+15,10*i+15);
        
        // P3
        particleShadows[i].t = 1;



        // boosting related stuff
        particleShadows[i].localMinimaReached = false;
        particleShadows[i].isBoostingActivated = false
        particleShadows[i].optimalCoverageH1 = particleShadows[i].getLocalCombinedCoverageLevel();
        particleShadows[i].optimalCoverageS1 = this.position;
        particleShadows[i].boostingIterationNumber = 0;
        particleShadows[i].coverageLevel = 0;
        
        particleShadows[i].nearbyNonReflexVerticesFound = [];
        particleShadows[i].verticesFoundSoFarRegistry = [];
        particleShadows[i].targetVertexAnchor = -1;
        particleShadows[i].targetVertexAnchorIndex = -1;



        particleShadows[i].diminishingStepSizeModeInitiated  = false;
        particleShadows[i].diminishingStepSizeIterationNumber = 0;
        particleShadows[i].pastTrackPoints = [new Point2(particleShadows[i].position.x,particleShadows[i].position.y)];
    }
    //trajectoryFollowButton.style("background-color", "grey");
	if(shadowsFollowingMode==1){
		shadowsFollowingMode=0;
		clearInterval(shadowsFollowingInterval);
		resetParticleHistory();
	}
    /////if(variableStepSizeMode && autoTuneLipschitzMode){
    if(autoTuneLipschitzMode){
        //////clearInterval(updateLocalLipschitzConstantsInterval);
        updateLocalLipschitzConstantsInterval = 0;
    }

    localMinimaReached = false;
    boostingActivated = 0;
    boostingIterationNumber = 0;

    optimalCoverageH1 = 0;
    optimalCoverageS1 = [];

	consolePrint("Resetted the Shadow Agents; Boosting Resetted.");

}


//reset double clicked
function resetAll(){

    linearSpeed = 0;
    isSimulationMode = false;
    
    //trajectoryFollowButton.style("background-color", "grey");
	if(shadowsFollowingMode==1){
		shadowsFollowingMode=0;
		clearInterval(shadowsFollowingInterval);
		resetParticleHistory();
	}
    if(autoTuneLipschitzMode){
	//////if(variableStepSizeMode && autoTuneLipschitzMode){
        //////clearInterval(updateLocalLipschitzConstantsInterval);
        updateLocalLipschitzConstantsInterval = 0;
    }
    for (var i = 0; i < particles.length; i++){//resetting positions 
        
    	//resetting particles
        particles[i].position = new Point2(10*i+15,10*i+15);
        
        //resetting particle shadows
        particleShadows[i].position = new Point2(10*i+15,10*i+15);

        //boosting related stuff
        particleShadows[i].localMinimaReached = false;
        particleShadows[i].isBoostingActivated = false
        particleShadows[i].optimalCoverageH1 = particleShadows[i].getLocalCombinedCoverageLevel();
        particleShadows[i].optimalCoverageS1 = particleShadows[i].position;
        particleShadows[i].boostingIterationNumber = 0;
        particleShadows[i].coverageLevel = 0;
        particleShadows[i].nearbyNonReflexVerticesFound = [];

        particleShadows[i].verticesFoundSoFarRegistry = [];
        particleShadows[i].targetVertexAnchor = -1;
        particleShadows[i].targetVertexAnchorIndex = -1;

        particleShadows[i].diminishingStepSizeModeInitiated  = false;
        particleShadows[i].diminishingStepSizeIterationNumber = 0;
    }
    
    // boosting related stuff
    localMinimaReached = false;
    boostingActivated = 0;
    boostingIterationNumber = 0;

    optimalCoverageH1 = 0;
    optimalCoverageS1 = [];

    consolePrint("Real Agents and Shadow Agents Returned to Home Position; Boosting Resetted.");

}



// stop particle button
function refreshAll(){
    location.reload(true);
}


function stopParticle(){
	
	isSimulationMode = false;
    linearSpeed = 0;
    
    if(shadowsFollowingMode==1){
    	pausedMode=1;
		shadowsFollowingMode=0;
		clearInterval(shadowsFollowingInterval);
		//resetParticleHistory();
	}
    ///////if(variableStepSizeMode && autoTuneLipschitzMode){
    if(autoTuneLipschitzMode){
        //////clearInterval(updateLocalLipschitzConstantsInterval);
        updateLocalLipschitzConstantsInterval = 0;
    }

    
    executionTime = Math.round(millis()-executionTime)/1000;
    consolePrint("Simulation Paused after "+executionTime+" s, Click 'Start' to Resume.");
    
    
   
}



function loadDefaultObstacle(arrayInput){
	
	obstacles[obstacles.length] = new Obstacle();
	


    var obstacleDropdown = document.getElementById("obstacleDropdown");
    var option1 = document.createElement("option");
    option1.text = nf(obstacles.length);
    
    var obstacleIndex = Number(obstacleDropdown.selectedIndex);
    
    obstacleDropdown.add(option1,obstacleIndex+1);
    obstacleDropdown.selectedIndex = obstacleIndex+1;
    
	 
	for(var i=0; i < arrayInput.length/2; i++){
		obstacles[obstacles.length-1].update(arrayInput[2*i],arrayInput[2*i+1]);
	}
	
	obstacles[obstacles.length-1].drawBoarder(obstacleColor);
	
}


function loadDefaultObstacle(arrayInput,invalidNonReflexVertices){
	
	obstacles[obstacles.length] = new Obstacle();
	


    var obstacleDropdown = document.getElementById("obstacleDropdown");
    var option1 = document.createElement("option");
    option1.text = nf(obstacles.length);
    
    var obstacleIndex = Number(obstacleDropdown.selectedIndex);
    
    obstacleDropdown.add(option1,obstacleIndex+1);
    obstacleDropdown.selectedIndex = obstacleIndex+1;
    
	 
	for(var i=0; i < arrayInput.length/2; i++){
		obstacles[obstacles.length-1].update(arrayInput[2*i],arrayInput[2*i+1]);
	}
	
	obstacles[obstacles.length-1].drawBoarder(obstacleColor);
	obstacles[obstacles.length-1].invalidNonReflexVertices = invalidNonReflexVertices;
	
}







// load multiple obstacles once
function obstacleSetupDropdownEvent(){
	//location.reload();
    var obstacleSetupDropdown = document.getElementById("obstacleSetupDropdown");
    var obstacleSetupIndex = Number(obstacleSetupDropdown.selectedIndex);
    //print("here");
    obstacleSetupLoad(obstacleSetupIndex);
    consolePrint("Obstacle Setup Changed.");
    
}

function selectDataToSaveDropdownEvent(){
    var selectDataToSaveDropdown = document.getElementById("selectDataToSaveDropdown");
    selectedDataToSave = Number(selectDataToSaveDropdown.selectedIndex);
}

function decentralizedBoostingMethodDropdownEvent(){
	decentralizedBoostingMethod = document.getElementById("decentralizedBoostingMethod").value;
	boostingMethodChanged();
    if(decentralizedBoostingMethod==0){
        document.getElementById("stepSize").value = 8;
    }else{
        document.getElementById("stepSize").value = 10;
    }
    consolePrint("Nature of the Boosting Method Changed.");
}


function obstacleDropdownEvent(){
	
    document.getElementById("obstacleCoordinatesDisplay").value = obstacles[document.getElementById("obstacleDropdown").value-1].textString;
	//assigned text
    consolePrint("Selected Obstacle Changed.");
}



function obstacleTextInput(){
    var newString = document.getElementById("obstacleCoordinatesDisplay").value;//what if string input is empty? - need to delete obstacle; //create default obstacles
	var strArray = splitTokens(newString, ",");
	
    var obstacleDropdown = document.getElementById("obstacleDropdown");
    	

    if(strArray.length==0){
        var obstacleIndex = Number(obstacleDropdown.selectedIndex);
        obstacles.splice(obstacleIndex,1);
        //document.getElementById("obstacleDropdown").selectedIndex = obstacles.length-1;
        
        for(var i = obstacleDropdown.options.length - 1 ; i >= 0 ; i--){
            
            document.getElementById("obstacleDropdown").remove(i);

        }


        for(var i = 0; i<obstacles.length;i++){
            var option1 = document.createElement("option");
            option1.text = nf(i+1);
            print(option1.text);
            document.getElementById("obstacleDropdown").add(option1,i);
            
            
        	obstacles[i].updateIndex(i);
            
        }

        
        if(obstacles.length>0){
            document.getElementById("obstacleDropdown").selectedIndex = obstacles.length-1;
            document.getElementById("obstacleCoordinatesDisplay").value = obstacles[obstacles.length-1].textString;
        }
    }
    else{
    	
        var obstacleIndex = Number(obstacleDropdown.value)-1;

    	obstacles[obstacleIndex].x = [];
    	obstacles[obstacleIndex].y = [];

    	for(var i=0; i < strArray.length/2; i++){
            obstacles[obstacleIndex].update(int(strArray[2*i]),int(strArray[2*i+1]));
    	}

    	obstacles[obstacleIndex].drawBoarder(obstacleColor);
    	obstacles[obstacleIndex].calculateReflexVertices();

    	//CASE WHERE new obstacle was added solely using coordinates
    	if(typeof(obstacles[obstacleIndex].nonReflexVertices[0]) == "undefined"){
    		obstacles[obstacleIndex].nonReflexVertices.splice(0,1);

    	}
    	
    	if(!obstacles[obstacleIndex].isConvex){//if found non convex obstacle
    		print("Reversing cordinates");
    		var tempX = obstacles[obstacleIndex].x;
    		var tempY = obstacles[obstacleIndex].y;
    		tempX.reverse();
    		tempY.reverse();
	    	obstacles[obstacleIndex].x = [];
	    	obstacles[obstacleIndex].y = [];

	    	for(var i=0; i < tempX.length; i++){
	            obstacles[obstacleIndex].update(tempX[i],tempY[i]);
	    	}

	    	obstacles[obstacleIndex].drawBoarder(obstacleColor);
	    	obstacles[obstacleIndex].calculateReflexVertices();
	    	if(!obstacles[obstacleIndex].isConvex){
	    		obstacles.splice(obstacleIndex,1);
	    		document.getElementById("obstacleCoordinatesDisplay").placeholder = "Error... Enter cordinates in a different order";
	    		print("Enter cordinates in a different order");

	    	}

    	}
    	consolePrint("Succesfully Modified the Obstacle");

    	

    }

    strokeWeight(1);
    for (var i=0; i < obstacles.length; i++){//draw obstacles
    	obstacles[i].drawBoarder(obstacleColor);
    }
    strokeWeight(4);
    

    loadPixels();
    pixelMap = pixels;
    print("pixels loaded");
    //numberOfObstaclesOld = obstacles.length;
    consolePrint("Obstacle Edited.")
    


}



function addObstacle(){
	
	
	if(addObstacleMode==0){//first time
		addObstacleMode = 1;
		document.getElementById("obstacleCoordinatesDisplay").value = "";
		document.getElementById("obstacleCoordinatesDisplay").placeholder = "Type Required Obstacle Coordinates Here...";
		
		cursor(CROSS);
	}
	else if(addObstacleMode==2){//after collecting vertexes
		addObstacleMode = 0;
		//addObstacleButton.style("background-color", "grey");
		
        obstacles[obstacles.length-1].calculateReflexVertices();
        var obstacleIndex = obstacles.length-1
		if(!obstacles[obstacleIndex].isConvex){//if found non convex obstacle
    		print("Reversing cordinates");
    		var tempX = obstacles[obstacleIndex].x;
    		var tempY = obstacles[obstacleIndex].y;
    		tempX.reverse();
    		tempY.reverse();
	    	obstacles[obstacleIndex].x = [];
	    	obstacles[obstacleIndex].y = [];

	    	for(var i=0; i < tempX.length; i++){
	            obstacles[obstacleIndex].update(tempX[i],tempY[i]);
	    	}

	    	obstacles[obstacleIndex].drawBoarder(obstacleColor);
	    	obstacles[obstacleIndex].calculateReflexVertices();
	    	if(!obstacles[obstacleIndex].isConvex){
	    		obstacles.splice(obstacleIndex,1);
	    		document.getElementById("obstacleCoordinatesDisplay").placeholder = "Error... Enter cordinates in a different order";
	    		print("Enter cordinates in a different order");

	    	}

		}
		cursor(ARROW);
	}
		
	//cnv.mouseClicked(updateVertex);
		
}


function stopAddingObstacle(){
	if(addObstacleMode==2){//after collecting vertexes
		addObstacleMode = 0;
		
        obstacles[obstacles.length-1].calculateReflexVertices();
        var obstacleIndex = obstacles.length-1
		if(!obstacles[obstacleIndex].isConvex){//if found non convex obstacle
    		print("Reversing cordinates");
    		var tempX = obstacles[obstacleIndex].x;
    		var tempY = obstacles[obstacleIndex].y;
    		tempX.reverse();
    		tempY.reverse();
	    	obstacles[obstacleIndex].x = [];
	    	obstacles[obstacleIndex].y = [];

	    	for(var i=0; i < tempX.length; i++){
	            obstacles[obstacleIndex].update(tempX[i],tempY[i]);
	    	}

	    	obstacles[obstacleIndex].drawBoarder(obstacleColor);
	    	obstacles[obstacleIndex].calculateReflexVertices();
	    	if(!obstacles[obstacleIndex].isConvex){
	    		obstacles.splice(obstacleIndex,1);
	    		document.getElementById("obstacleCoordinatesDisplay").placeholder = "Error... Enter cordinates in a different order";
	    		print("Enter cordinates in a different order");

	    	}

		}
		cursor(ARROW);
	}
}



function trajectoryFollowButtonFunction(){
	//trajectoryFollowButton.style("background-color", "red");
	if(trajectoryFollowMode==0){//first time
		trajectoryFollowMode = 1;
		trjectoryFollowingInterval = setInterval(followTrajectory,100);
		
	}
	else{
		trajectoryFollowMode = 0;
		//trajectoryFollowButton.style("background-color", "grey");
		clearInterval(trjectoryFollowingInterval);
	}
	
	
	
	//particles[0].followTrajectory();
}



function pickWayPointsButtonFunction(){
	//pickWayPointsButton.style("background-color", "red");
	if(pickWayPointsMode==0){//first time
		pickWayPointsMode = 1;
		cursor(CROSS);
	}
	else if(pickWayPointsMode==2){
		pickWayPointsMode = 0;
		//pickWayPointsButton.style("background-color", "grey");
		cursor(ARROW);
	}
	
	
	
	//particles[0].followTrajectory();
}






function mouseClicked(){
	
	
	if(addObstacleMode>=1){//inserting obstacle vertexes
		
		if(mouseX>0 && mouseY>0 && mouseX<width && mouseY<height){
			//print(mouseX,mouseY);
			obstacles[obstacles.length-1].update(mouseX,mouseY);
			//obstacles[obstacles.length-1].updateBoarder();
			consolePrint("Vertex Point Added to the New Obstacle");
		}
		else{
			if(addObstacleMode==1){// just started by clicking the button
				obstacles[obstacles.length] = new Obstacle();

                var option1 = document.createElement("option");
                option1.text = nf(obstacles.length);
                document.getElementById("obstacleDropdown").add(option1);
                document.getElementById("obstacleDropdown").selectedIndex = obstacles.length-1;
				
				addObstacleMode = 2;//vertexex collecting mode
				consolePrint("Add New Obstacle - Type Vertices or Point Them in Workspace");
			}
			else{//clicked outside
				addObstacleMode = 0;
				
				obstacles[obstacles.length-1].calculateReflexVertices();
				var obstacleIndex = obstacles.length-1
				if(!obstacles[obstacleIndex].isConvex){//if found non convex obstacle
		    		print("Reversing cordinates");
		    		var tempX = obstacles[obstacleIndex].x;
		    		var tempY = obstacles[obstacleIndex].y;
		    		tempX.reverse();
		    		tempY.reverse();
			    	obstacles[obstacleIndex].x = [];
			    	obstacles[obstacleIndex].y = [];

			    	for(var i=0; i < tempX.length; i++){
			            obstacles[obstacleIndex].update(tempX[i],tempY[i]);
			    	}

			    	obstacles[obstacleIndex].drawBoarder(obstacleColor);
			    	obstacles[obstacleIndex].calculateReflexVertices();
			    	if(!obstacles[obstacleIndex].isConvex){
			    		obstacles.splice(obstacleIndex,1);
			    		document.getElementById("obstacleCoordinatesDisplay").placeholder = "Error... Enter cordinates in a different order";
			    		print("Enter cordinates in a different order");

			    	}

    			}
				cursor(ARROW);
				consolePrint("Finished Adding Obstacles");
			}
		}
	
	}
	
	else if(pickWayPointsMode>=1){
		if(mouseX>0 && mouseY>0 && mouseX<width && mouseY<height){
			append(particles[0].wayPoints,new Point2(mouseX,mouseY));
			print(mouseX,mouseY);
		}
		else{
			if(pickWayPointsMode==1){//just started
				//finish collecting
				pickWayPointsMode = 2;
				cursor(CROSS);
			}
			else{
				pickWayPointsMode = 0;
				//pickWayPointsButton.style("background-color", "grey");
				cursor(ARROW);
				//print("hgghyg");
				particles[0].generateReferencePointTrajectory();
			}
		}
		
	}
	
	
}



//this function adds a new particle at the mouse location whenever the "+" key is pressed and released

function keyReleased() {

	if(keyCode==107){
		particles.push(new Particle(mouseX, mouseY));
		particleShadows.push(new Particle(mouseX, mouseY));
	}

}



function addAgent(){

    //particles.push(new Particle(20*particles.length+20, 20*particles.length+20));
	particles.push(new Particle(10*particles.length+15, 10*particles.length+15));
    particleShadows.push(new Particle(10*particleShadows.length+15, 10*particleShadows.length+15));
	////particleShadows.push(new Particle(10, 10));
	//trajectoryFollowButton.style("background-color", "grey");
	if(shadowsFollowingMode==1){
		shadowsFollowingMode=0;
		clearInterval(shadowsFollowingInterval);
		resetParticleHistory();
	}

    //// creating drop down element
    if(submodularityMode!=2){
    	var option1 = document.createElement("option");
    	option1.text = nf(particleShadows.length);
    	document.getElementById("particleSelectDropdown").add(option1);
    	document.getElementById("particleSelectDropdown").selectedIndex = particleShadows.length;
    	particleShadows[particleShadows.length-1].senRange = Number(document.getElementById("sensingRange").value);
    	particleShadows[particleShadows.length-1].sensingDecayFactor = Number(document.getElementById("sensingDecay").value)/1000;
        particleShadows[particleShadows.length-1].updateSensingCapacity();
        particleShadows[particleShadows.length-1].agentCostRatio = Number(document.getElementById("agentCostRatio").value)/1000;
    }
    //// end creating drop down element


	consolePrint("Agent Added.");
}

function addAgentToPoint(x,y){

    //particles.push(new Particle(20*particles.length+20, 20*particles.length+20));
    particles.push(new Particle(x,y));
    particleShadows.push(new Particle(x,y));
    ////particleShadows.push(new Particle(10, 10));
    //trajectoryFollowButton.style("background-color", "grey");
    if(shadowsFollowingMode==1){
        shadowsFollowingMode=0;
        clearInterval(shadowsFollowingInterval);
        resetParticleHistory();
    }

    //consolePrint("Agent Added.");
}

function addClassifiedAgentToPoint(originalAgentIndex,x,y){

    //particles.push(new Particle(20*particles.length+20, 20*particles.length+20));
    var newParticle = savedParticles[originalAgentIndex];
    newParticle.x = x;
    newParticle.y = y;
    newParticle.position = new Point2(x,y);

    particles.push(newParticle);
    particleShadows.push(newParticle);
    ////particleShadows.push(new Particle(10, 10));
    //trajectoryFollowButton.style("background-color", "grey");
    if(shadowsFollowingMode==1){
        shadowsFollowingMode=0;
        clearInterval(shadowsFollowingInterval);
        resetParticleHistory();
    }

    
}


function addClassifiedAgentToPointFinal(originalAgentIndex,x,y){

    //particles.push(new Particle(20*particles.length+20, 20*particles.length+20));
    var newParticle = savedParticles[originalAgentIndex];
    newParticle.x = x;
    newParticle.y = y;
    newParticle.position = new Point2(x,y);

    particles.push(newParticle);
    particleShadows.push(newParticle);
    ////particleShadows.push(new Particle(10, 10));
    //trajectoryFollowButton.style("background-color", "grey");
    if(shadowsFollowingMode==1){
        shadowsFollowingMode=0;
        clearInterval(shadowsFollowingInterval);
        resetParticleHistory();
    }



    //// creating drop down element
    if(submodularityMode!=2){ 
        var option1 = document.createElement("option"); 
        option1.text = nf(originalAgentIndex+1);
        document.getElementById("particleSelectDropdown").add(option1);
        document.getElementById("particleSelectDropdown").selectedIndex = originalAgentIndex+1;
        document.getElementById("sensingRange").value = savedParticles[originalAgentIndex].senRange;
        document.getElementById("sensingDecay").value = 1000*savedParticles[originalAgentIndex].sensingDecayFactor;
        document.getElementById("agentCostRatio").value = 1000*savedParticles[originalAgentIndex].agentCostRatio;
        ////document.getElementById("agentCostRatio").value = Math.round(savedParticles[originalAgentIndex].agentCostRatio*savedParticles[originalAgentIndex].sensingCapacity);
    }
    //// end creating drop down element

    ////consolePrint("Agent "+(originalAgentIndex+1)+" Added at ("+x+" , "+y+")");
}



function removeAgent(){

    particles.splice(particles.length-1,1);
    particleShadows.splice(particleShadows.length-1,1);
    //trajectoryFollowButton.style("background-color", "grey");
	if(shadowsFollowingMode==1){
		shadowsFollowingMode=0;
		clearInterval(shadowsFollowingInterval);
		resetParticleHistory();
	}

	if(submodularityMode!=2){
		document.getElementById("particleSelectDropdown").remove(particleShadows.length+1);
        document.getElementById("particleSelectDropdown").selectedIndex = particleShadows.length;
	}
	//consolePrint("Agent Removed.");
    
}

function removeFirstAgent(){

    particles.shift();
    particleShadows.shift();
    
}


//This function moves the particles as we drag them

function mouseDragged() {

    /*for (var i = 0; i < particles.length; i++) {
        particles[i].clicked();
    }*/
	staticMode=0;
	countTillStaticMode=0;
	//trajectoryFollowButton.style("background-color", "grey");
	if(shadowsFollowingMode==1){
		shadowsFollowingMode=0;
		clearInterval(shadowsFollowingInterval);
		resetParticleHistory();
	}


	
	if(particleDraggingMode){
		consolePrint("Dragging Particles");
		if(particleDragging>=0){
			particleShadows[particleDragging].position = new Point2(mouseX,mouseY);
		}else if(particleDragging<0){
			particles[-1*particleDragging-1].position = new Point2(mouseX,mouseY);
		}
	}
	else{
		
		for (var i = 0; i < particleShadows.length; i++) {
			if(particleShadows[i].clicked()){
				particleDragging = i;
				particleDraggingMode = true;
			}
			/*else if(particles[i].clicked()){
				particleDragging = -1*(i+1);
				particleDraggingMode = true;	
			}*/
			
		}
	}	
	
	
	
	
}


function mouseReleased() {
	
	if(particleDraggingMode){
		particleDraggingMode = false;
		if(particleDragging>=0){
			particleShadows[particleDragging].position = new Point2(mouseX,mouseY);
		}else if(particleDragging<0){
			if(pausedMode){
				resetParticleHistory2();//have to do something here!!!
				print("special");
			}
			particles[-1*particleDragging-1].position = new Point2(mouseX,mouseY);
			
		}
		consolePrint("Dragging Finished")
	}
}


function boostingMethodChanged(){
	boostingMethod = Number(document.getElementById("boostingMethod").selectedIndex);
	consolePrint("Boosting Method Changed Into: "+boostingMethod+".");
	
	//centralized approach
    //////clearInterval(testLocalMinimaReachedInterval);
	testLocalMinimaReachedInterval = 0;
    //////clearInterval(testAgentLocalMinimaReachedInterval);
	testAgentLocalMinimaReachedInterval = 0;
    testJustConvergenceInterval = 0;

    for(var i=0;i<particleShadows.length;i++){// reset past data
        particleShadows[i].derivativeSumValue = new Point2(0,0);
    }

		
	if(boostingMethod>0){
		
		if(decentralizedBoostingMethod==0){
			print("cb-m0");
            boostingActivated==0;
            //////testLocalMinimaReachedInterval = setInterval(testLocalMinimaReached,5000);
			testLocalMinimaReachedInterval = 50; //once every 50 iterations

		}else if(decentralizedBoostingMethod==1){// for decentralized approaches
			print("db-m1");
            
            for(var i = 0; i<particleShadows.length;i++){
                particleShadows[i].isBoostingActivated = 0;
            }
            //////testAgentLocalMinimaReachedInterval = setInterval(testAgentLocalMinimaReached,5000);
			testAgentLocalMinimaReachedInterval = 50; //once every 50 iterations
		}
		
	}
	else{
        testJustConvergenceInterval = 50;
		print("cancelled all boosting");
	}


    // loading boosting parameters
	document.getElementById("boostingParameterY").disabled = false;
	if(boostingMethod==0){//no boosting
		boostingParameterK = 0;
		boostingParameterY = 0;
	}else if(boostingMethod==1){// P 
		boostingParameterK = 1;
		boostingParameterY = 1;
	}else if(boostingMethod==2){// nei
		boostingParameterK = 10000;
		boostingParameterY = 1;
	}else if(boostingMethod==3){// Phi
		boostingParameterK = 4;
		boostingParameterY = 2;
	}else if(boostingMethod==4){// random
		boostingParameterK = 5;
		boostingParameterY = 0;
		document.getElementById("boostingParameterY").disabled = true;
	}else if(boostingMethod==5){
        boostingParameterK = [10,5];
        boostingParameterY = [1,1];
    }else if(boostingMethod==6){
        boostingParameterK = 1;
        boostingParameterY = 1;
    }else if(boostingMethod==7){
        boostingParameterK = 1;
        boostingParameterY = 1;
    }
    document.getElementById("boostingParameterK").value = boostingParameterK;
    document.getElementById("boostingParameterY").value = boostingParameterY;
    document.getElementById("boostingK").innerHTML = "&kappa;:";
    document.getElementById("boostingGamma").innerHTML = "&gamma;:";

    if(boostingMethod==5){// V
		document.getElementById("boostingK").innerHTML = "&kappa;<sub>1</sub>, &kappa;<sub>2</sub> :";
        document.getElementById("boostingGamma").innerHTML = "&gamma;<sub>1</sub>, &gamma;<sub>2</sub> :";
	}
    
	consolePrint("Default Boosting Parameters Loaded.")
	boostingActivated = 0;
}


function boostingParameterChanged(){
	//boostingMethod = Number(document.getElementById("boostingMethod").selectedIndex);

	boostingParameterK = Number(document.getElementById("boostingParameterK").value);
	boostingParameterY = Number(document.getElementById("boostingParameterY").value);
    boostingMethod = Number(document.getElementById("boostingMethod").selectedIndex);
    if(boostingMethod==5){// V
        boostingParameterK = document.getElementById("boostingParameterK").value.split(',').map(Number);
        boostingParameterY = document.getElementById("boostingParameterY").value.split(',').map(Number);
        print(boostingParameterK)
    }
    consolePrint("Boosting Parameters Changed.")
}

function variableStepSizeModeChanged(){
    variableStepSizeMode = document.getElementById("variableStepSizeCB").checked;
    if(variableStepSizeMode){
        var stringText = document.getElementById("stepSizeDisplayLabel").innerHTML;
        var cropRemains = stringText.slice(-1*stringText.length+9);
        //print(cropRemains);
        document.getElementById("stepSizeDisplayLabel").innerHTML = "Beta Gain"+ cropRemains; 

    }else{
        var stringText = document.getElementById("stepSizeDisplayLabel").innerHTML;
        var cropRemains = stringText.slice(-1*stringText.length+9);
        //print(cropRemains);
        document.getElementById("stepSizeDisplayLabel").innerHTML = "Step Size"+ cropRemains; 
        
    }
    
    for(var i = 0; i<particleShadows.length; i++){
        particleShadows[i].variableStepSizeMode = variableStepSizeMode;
        particleShadows[i].variableStepSizeModeDisabled = !variableStepSizeMode;
    }
    autoTuneLipschitzModeChanged();
}

function autoTuneLipschitzModeChanged(){
    
    autoTuneLipschitzMode = document.getElementById("autoTuneLipschitzMode").checked;
    ///////if(autoTuneLipschitzMode && variableStepSizeMode){
    if(autoTuneLipschitzMode){
        tuneLipschitzConstantsLocally();
        //////updateLocalLipschitzConstantsInterval = setInterval(tuneLipschitzConstantsLocally,5000);
        updateLocalLipschitzConstantsInterval = 100;
    }else{
        print("here");
        //////clearInterval(updateLocalLipschitzConstantsInterval);
        updateLocalLipschitzConstantsInterval = 0;
        //tuneLipschitzConstants();
    }

}

function diminishingStepSizeModeChanged(){
    diminishingStepSizeMode = document.getElementById("diminishingStepSizeCB").checked;
    if(diminishingStepSizeMode){
        
        document.getElementById("diminishingStepSizerSlider").disabled = false;
        document.getElementById("diminishingStepSizerDisplay").disabled = false;
        document.getElementById("diminishingStepSizeInitialStep").disabled = false;


        if(document.getElementById("diminishingStepSizeInitialStep").value==""){
            diminishingStepSizeK0 = round(1000/lipschitzConstantK1)/1000;
            document.getElementById("diminishingStepSizeInitialStep").value = diminishingStepSizeK0;
        }else if(Number(document.getElementById("diminishingStepSizeInitialStep").value)!=(1/lipschitzConstantK1)){
            print("Custom K_0")
            diminishingStepSizeK0 = Number(document.getElementById("diminishingStepSizeInitialStep").value);
        }else{//redundent actually
            diminishingStepSizeK0 = round(1000/lipschitzConstantK1)/1000;
            document.getElementById("diminishingStepSizeInitialStep").value = diminishingStepSizeK0;
        }

        diminishingStepSizer = Number(document.getElementById("diminishingStepSizerSlider").value)/100;
        document.getElementById("diminishingStepSizerDisplay").innerHTML = diminishingStepSizer;
    }
    else{

        document.getElementById("diminishingStepSizerSlider").disabled = true;
        document.getElementById("diminishingStepSizerDisplay").disabled = true;
        document.getElementById("diminishingStepSizeInitialStep").disabled = true;
        // deactivate the text field menu
        // deactivate the slider
        // deactivate the text fields related to r and K_0

    }
    for(var i = 0; i<particleShadows.length; i++){
        particleShadows[i].diminishingStepSizeMode = diminishingStepSizeMode;
        particleShadows[i].diminishingStepSizeModeInitiated = false;
    }

}



/*function weightPGDChanged(){
    weightPGD = Number(document.getElementById("weightPGDDisplay").value);    
    print("Weight PGD Changed to "+weightPGD);
}*/

function normalizationFactorChanged(val){
    var normalizationFactor = val/1000000;
    var sumNum = 0; 
    var sumDen = 0;
    for(var i = 0; i<particleShadows.length; i++){
        sumNum = sumNum + particleShadows[i].sensingCapacity;
        sumDen = sumDen + particleShadows[i].agentCostRatio*particleShadows[i].sensingCapacity
    }
    weightPGD = -1*(sumNum/sumDen)*(normalizationFactor/(1-normalizationFactor));
}


function particleSelectDropdownEvent(){
    
    //// adjust sensing range and decay sliders
	var val = Number(document.getElementById("particleSelectDropdown").value); 
    if(isNaN(val)){
        print("Resetting all agents");
    	for(var i = 0; i<particleShadows.length; i++){// all agents reset to default
    		particleShadows[i].senRange = senRange;
    		particleShadows[i].sensingDecayFactor = sensingDecayFactor;

            particleShadows[i].updateSensingCapacity();// updates the sensing capacity
            particleShadows[i].agentCostRatio = agentCostRatio; // updates the cost ratio
    	}
    	document.getElementById("sensingRange").value = senRange;
   		document.getElementById("sensingDecay").value = sensingDecayFactor*1000;
        document.getElementById("agentCostRatio").value = agentCostRatio*1000;
    }else{// just display
        print("Showing Agent "+val+" config");
    	document.getElementById("sensingRange").value = particleShadows[val-1].senRange;
   		document.getElementById("sensingDecay").value = particleShadows[val-1].sensingDecayFactor*1000;
        document.getElementById("agentCostRatio").value = particleShadows[val-1].agentCostRatio*1000;
    }
    //assigned text;
    //assigned text
    consolePrint("Selected agent changed.");
    particleSelectDropdownUsed = true;
}

function sensingRangeChanged(val){
    if(particleSelectDropdownUsed){
        particleSelectDropdownUsed = false;
    }else{
    	var val1 = Number(document.getElementById("particleSelectDropdown").value); 
	    if(isNaN(val1)){
	    	for(var i = 0; i<particleShadows.length; i++){// all agents reset to default
	    		particleShadows[i].senRange = val;

                particleShadows[i].updateSensingCapacity();
	    	}
	    }else{
        	particleShadows[val1-1].senRange = val;

            particleShadows[val1-1].updateSensingCapacity(); 
    	}
    }
}

function sensingDecayChanged(val){
    if(particleSelectDropdownUsed){
        particleSelectDropdownUsed = false;
    }else{
    	var val1 = Number(document.getElementById("particleSelectDropdown").value); 
	    if(isNaN(val1)){
	    	for(var i = 0; i<particleShadows.length; i++){// all agents reset to default
	    		particleShadows[i].sensingDecayFactor = val/1000;

                particleShadows[i].updateSensingCapacity();
	    	}
	    }else{
        	particleShadows[val1-1].sensingDecayFactor = val/1000;

            particleShadows[val1-1].updateSensingCapacity(); 
    	}
        
    }
}


function agentCostRatioChanged(val){
    if(particleSelectDropdownUsed){
        particleSelectDropdownUsed = false;
    }else{
        var val1 = Number(document.getElementById("particleSelectDropdown").value); 
        if(isNaN(val1)){
            for(var i = 0; i<particleShadows.length; i++){// all agents reset to default
                particleShadows[i].agentCostRatio = val/1000;
            }
        }else{
            particleShadows[val1-1].agentCostRatio = val/1000; 
            print("Agent "+val1+" cost ratio changed");
        }
        
    }
}




function draw() {

	//slider reading
	senRange = Number(document.getElementById("sensingRange").value);
    document.getElementById("sensingRangeDisplay").innerHTML = senRange;
        
    sensingDecayFactor = Number(document.getElementById("sensingDecay").value)/1000;
    document.getElementById("sensingDecayDisplay").innerHTML = sensingDecayFactor;

    if(particleShadows.length>0 && submodularityMode<=1){

        agentCostRatio = Number(document.getElementById("agentCostRatio").value)/1000; /// same as partilceShadows[selectedAgentIndex].agentCostRatio
        document.getElementById("agentCostRatioDisplay").innerHTML = agentCostRatio;
        var selectedAgentIndex = Number(document.getElementById("particleSelectDropdown").value);
        if(isNaN(selectedAgentIndex)){selectedAgentIndex=1;}
        var agentCost = agentCostRatio*particleShadows[selectedAgentIndex-1].sensingCapacity;
        document.getElementById("agentCostDisplay").innerHTML = Math.round(agentCost);


        var normalizationFactor = Number(document.getElementById("normalizationFactor").value)/1000000;
        document.getElementById("normalizationFactorDisplay").innerHTML = normalizationFactor.toFixed(3);
        var sumNum = 0; 
        var sumDen = 0;
        for(var i = 0; i<particleShadows.length; i++){
            sumNum = sumNum + particleShadows[i].sensingCapacity;
            sumDen = sumDen + particleShadows[i].agentCostRatio*particleShadows[i].sensingCapacity;
        }
        weightPGD = -1*(sumNum/sumDen)*(normalizationFactor/(1-normalizationFactor));
        // print("weightPGD: "+weightPGD);
        // print("weightPGD*alpha_i: "+weightPGD*agentCostRatio*particleShadows[selectedAgentIndex-1].sensingCapacity);
        document.getElementById("weightPGDDisplay").innerHTML = Math.round(-1*weightPGD*1000)/1000;
    }


    descretizationLevel = Number(document.getElementById("descretizationLevelVal").value);
    document.getElementById("descretizationLevelDisplay").innerHTML = descretizationLevel;        

    hStep = Number(document.getElementById("stepSizeMultiplier").value);
    document.getElementById("stepSizeMultiplierDisplay").innerHTML = hStep;

    linearSpeed = Number(document.getElementById("stepSize").value);
    constantStepSize = round(100*(linearSpeed*hStep))/100;
    document.getElementById("stepSizeDisplay").innerHTML = linearSpeed+" x "+hStep+" = "+constantStepSize;    


    displayPhysicalAgentsMode = document.getElementById("displayPhysicalAgents").checked;
    saveDataMode = document.getElementById("saveDataEnabled").checked;
    
    plotDataMode = document.getElementById("plotDataEnabled").checked;

    
    variableStepSizeMode = document.getElementById("variableStepSizeCB").checked;
    negativeStepsAllowedMode = document.getElementById("negativeStepSizeCB").checked;

    diminishingStepSizeMode = document.getElementById("diminishingStepSizeCB").checked;
    if(diminishingStepSizeMode){
        diminishingStepSizer =Number(document.getElementById("diminishingStepSizerSlider").value)/100;
        document.getElementById("diminishingStepSizerDisplay").innerHTML = diminishingStepSizer;
    }

    




    //drawing basic environment
    background(255);
    strokeWeight(4);
    noFill();
    stroke(0);
    rect(0,0,width,height);
    
    // PSO Display
    for (var i = 0; i < pointsToBePrinted.length; i++) {
        printPointPSO(pointsToBePrinted[i],solutionOfPSO[i].ID,30);
    }
    
    
    if(addObstacleMode >= 1){//print cursor coordinate
    	var coordinateString = [nf(round(mouseX)),nf(round(mouseY))];
    	fill(0);
    	strokeWeight(1);
    	if(obstacles.length>0){
    		if(obstacles[obstacles.length-1].x.length>1){
    			line(mouseX,mouseY, obstacles[obstacles.length-1].x[obstacles[obstacles.length-1].x.length-1], obstacles[obstacles.length-1].y[obstacles[obstacles.length-1].y.length-1] );
    		}
    	}
    	text("Pick Next Vertex:", mouseX+5, mouseY-15);
    	text(coordinateString, mouseX+5, mouseY+15);
    }
    

    strokeWeight(1);
    for (var i=0; i < obstacles.length; i++){//draw obstacles
    	obstacles[i].drawBoarder(obstacleColor);
    }
    strokeWeight(4);
 
 	//storing pixel map for future reference
    if((addObstacleMode==0) && ((pixelMap.length == 0) || ((obstacles.length-numberOfObstaclesOld)!=0) )){
    	loadPixels();
    	pixelMap = pixels;
    	print("pixels loaded");
    	numberOfObstaclesOld = obstacles.length;
    }


    drawMinCostFlowSolution();


    drawSensingColorMap(10); //pixelSize = 10

    
    
    //update particleShadows
    //var timeStart = millis();
    
    // for Q investigation
    if(variableStepSizeMode  && saveDataMode){
        DeltaValuesMatrix = math.zeros(particleShadows.length,particleShadows.length)._data;
        // values will be loaded later
    }

    // to avoid agent flocks in variable step size method
    

    // for plotting
    derivativeValues = [];
    betaValues = [];
    QValues = [];


    // update 	
    if(addObstacleMode==0 && !isDebugMode && isSimulationMode && !mouseIsPressed){
        
        // find the next position to be....
        var startTime = millis();
        var weight = -15000;
        var sum_t = 0;

        for (var i = 0; i < particleShadows.length; i++) {
            //particleShadows[i].updateNew(i);
            if(variableStepSizeMode){
                particleShadows[i].variableStepSizeUpdate(i);
            }
            else{
                particleShadows[i].updateNew(i);
                ////P3
                sum_t = sum_t + particleShadows[i].t;
                print("index is "+ i + ",t is " + particleShadows[i].t);
                ////P3
            }
        }
        // end - find the next position to be....

        // executing all the steps !!!
        for (var i = 0; i < particleShadows.length; i++) {
            particleShadows[i].executeUpdate();
        }
        // end - executing all the steps !!!

        ////P3
        executionTimePGD =  executionTimePGD + millis() - startTime;
        print("TimePGD : "+executionTimePGD);
        var agent_cost = -weight*sum_t;
        print("sum_t is :" + sum_t + " ,agent_cost is: " + agent_cost);
        ////P3

        stateUpdateIterationNumber++;
    }


    // Display     
    for(var i = 0; i < particleShadows.length; i++){
        
        particleShadows[i].showShadow();

        if(displayPhysicalAgentsMode){
            particles[i].show(i);
        }
    }


    /////////////////////////
    // submodularity display candidates
    if(submodularityMode==1){
        //print("dvdvd");
        printPointArrayP2(submodularityCandidates,color(100,0,100),2);
        //printPoint2Array(submodularityCandidates,color(100,0,100),20);
    }else if(submodularityMode==2){// need to simulate the vcandidate points
        
        // update
        // find the next position to be....
        for (var i = 0; i < particleShadows.length; i++) {
            particleShadows[i].quickUpdate(10);// argument is the step size suppressing factor
            //particleShadows[i].variableStepSizeUpdate(i);
        }
        // end - find the next position to be....

        // execute
        // executing all the steps !!!
        var totalDisplacement = 0;
        for (var i = 0; i < particleShadows.length; i++) {
            particleShadows[i].executeUpdate();
            var distance =  distP2(particleShadows[i].lastPosition,particleShadows[i].position);
            totalDisplacement = totalDisplacement + distance;
        }
        print(totalDisplacement,savedParameters[2]);
        savedParameters[2] = savedParameters[2]-1;
        // end - executing all the steps !!!

        // Display 
        if(totalDisplacement<2||savedParameters[2]==0){    
            for(var i = 0; i < particleShadows.length; i++){
                submodularityCandidates[i] = particleShadows[i].position; 
            }
            printPointArrayP2(submodularityCandidates,color(100,0,100),2);
            readjustCandidatesEnd();
        }

    }else if(submodularityMode==3){//simulate centralized - general greedy
        document.getElementById("displayCoverageDensity").checked = true;
        printPointArrayP2(submodularityCandidates,color(100,0,100),2);    
        if(savedParticles.length>savedParameters[1]){// need to add an agent
			
			var startTime = millis();
            iterationOfCentralizedGreedy();// add the most benificial agent
			executionTimeGreedy = executionTimeGreedy + millis()-startTime;
			
            consolePrint("Agent "+(particleShadows[particleShadows.length-1].id+1)+" in class "+(savedParameters[6][savedParameters[6].length-1]+1)+" added, Global cost:"+Math.round(globalObjective()));
        }else{
            submodularityMode = 1;
            var approxFactor = 1-math.pow(1-(1/savedParticles.length),savedParticles.length);//1-(1-1/N)^N)
            if(approxFactor<0){approxFactor=0;}
            var approxFactor2 = savedParameters[3].reduce(function(a, b){return Math.max(a, b);});
            approxFactor2 = (1-approxFactor2)*(1-(1/savedParticles.length));
            consolePrint("Solving Using Centralized - General Greedy Algorithm: Finished after "+savedParameters[2]+" computations.");
            consolePrint("Approximation factors (Online computed) : Theoretical :- "+approxFactor+", Greedy Curvature Based :- "+approxFactor2+".");
			consolePrint("executionTimeGreedy time = "+executionTimeGreedy);

            //// Start code for the CDC papr 3 with chongchong
            ////consolePrint("Special Print Section Started...");
            calculateApproxFactorsBtnFcn();
  /*          var stringValue = "Final Agent Coordinates: ";
            for(var i = 0; i < particleShadows.length; i++){
                stringValue = stringValue + "("+particleShadows[i].position.x.toString()+","+particleShadows[i].position.y.toString()+")"; 
            }
            consolePrint(stringValue);*/
            ////consolePrint("Special Print Section End");
            //// End code for the CDC papr 3 with chongchong 








        }
        printPointArrayP2(submodularityCandidates,color(100,0,100),2);
        //////sleepFor(200);// for better display

    }else if(submodularityMode==3.5){//simulate centralized - general greedy
        document.getElementById("displayCoverageDensity").checked = true;
        printPointArrayP2(submodularityCandidates,color(100,0,100),2);    
        if(savedParticles.length>savedParameters[1]){// need to add an agent
            iterationOfCentralizedGreedyContinuous();// add the most benificial agent
            consolePrint("Agent "+savedParameters[1]+" added, Global cost:"+Math.round(globalObjective()));
        }else if(savedParticles.length==savedParameters[1] && savedParameters[3]<savedParticles.length){
            // remove the first agent
            removeFirstAgent();
            savedParameters[1] = savedParameters[1] - 1; // current number of particles
            var sol1 = savedParameters[0][0]; // remove 
            savedParameters[0].shift(); // selected indexes
            savedParameters[4].shift(); // selected max _i
            iterationOfCentralizedGreedyContinuous();
            if(sol1==savedParameters[0][savedParameters[0].length-1]){
                savedParameters[3] = savedParameters[3]+1;
                consolePrint("Agent not adjusted, Global cost:"+Math.round(globalObjective()));
            }else{
                savedParameters[3] = 0;
                consolePrint("Agent adjusted, Global cost:"+Math.round(globalObjective()));
            }
        }else{
            submodularityMode = 1;
            var approxFactor = 0.632;//1-1/e
            if(approxFactor<0){approxFactor=0;}
            var approxFactor2 = savedParameters[4].reduce(function(a, b){return Math.max(a, b);});
            approxFactor2 = (1-approxFactor2)*(1-(1/savedParticles.length));
            consolePrint("Solving Using Centralized - General Greedy Algorithm: Finished after "+savedParameters[2]+" computations.");
            consolePrint("Approximation factors : Theoretical :- "+approxFactor+", Greedy Curvature Based :- "+approxFactor2+".");
        }
        printPointArrayP2(submodularityCandidates,color(100,0,100),2);
        sleepFor(200);// for better display

    }else if(submodularityMode==4){//simulate centralized - stochastic greedy
        document.getElementById("displayCoverageDensity").checked = true;
        if(savedParticles.length>savedParameters[1]){// need to add an agent
            iterationOfCentralizedStochasticGreedy();// add the most benificial agent
            consolePrint("Agent "+savedParameters[1]+" added, Global cost:"+Math.round(globalObjective()));
        }else{
            submodularityMode = 1;
            var epsilon = Number(document.getElementById("epsilonStochasticGreedy").value);
            var approxFactor = Math.round((0.63212055882-epsilon)*1000)/1000;
            if(approxFactor<0){approxFactor=0;}
            consolePrint("Solving Using Centralized - Stochastic Greedy Algorithm: Finished after "+savedParameters[2]+" computations, with avg. approx. factor: "+approxFactor+".");
        }
        printPointArrayP2(savedParameters[3],color(100,0,100),2);
        sleepFor(200);// for better display

    }else if(submodularityMode==5){//simulate distrbuted - randomized greedy
        document.getElementById("displayCoverageDensity").checked = true;
        var numOfMachines = Number(document.getElementById("mRandomizedGreedy").value);//m values
        var currentMachine = savedParameters[4];

        if(currentMachine<numOfMachines){// have we completed the calculations for all the machines   ?     
            
            if(savedParticles.length>savedParameters[1]){// need to add an agent
                iterationOfDistributedRandomizedGreedy(currentMachine);// add the most benificial agent
                var costValue = Math.round(globalObjective());
                consolePrint("Agent "+savedParameters[1]+" added to Machine "+(currentMachine+1)+", Global cost: "+costValue+".");
                if(savedParticles.length==savedParameters[1]){savedParameters[6].push(costValue);}
            }else{
                //pause(1000);
                // finished one machine, moving onto the next
                particleShadows = [];particles = []; // remove all the agents
                savedParameters[1] = 0; // length of current particle shadows
                savedParameters[4] = savedParameters[4] + 1; // current machine number
                if(savedParameters[4] == numOfMachines){//(all machines are finished)at the last stage before centralizing
                    // combine the indexes in savedParameters[0]
                    var reducedIndexes = [];
                    var reducedCandidates = []; //display purposes
                    for(var i=0;i<savedParameters[0].length;i++){
                        reducedIndexes = concat(reducedIndexes,savedParameters[0][i]);
                        for(var j =0; j<savedParameters[0][i].length; j++){
                            reducedCandidates.push(submodularityCandidates[savedParameters[0][i][j]]);
                        }
                    }
                    // last machine corresponds to the centralized computer
                    savedParameters[5][numOfMachines] = reducedIndexes;
                    savedParameters[0][numOfMachines] = [];
                    savedParameters[3][numOfMachines] = reducedCandidates;
                }
                consolePrint("Solving Usal Greedy Algorithm at machine "+(currentMachine+1)+": Finished. Moving to the next machine...");
            }
            printPointArrayP2(savedParameters[3][currentMachine],color(100,0,100),2);
            sleepFor(200);// for better display
        }else{
            consolePrint("All Machines Finished, Starting the centralized stage...");
            printPointArrayP2(savedParameters[3][currentMachine],color(100,0,100),2);

            if(savedParticles.length>savedParameters[1]){// need to add an agent (centralized stage)
                centralizedIterationOfTheDistributedRandomizedGreedy(numOfMachines);// add the most benificial agent
                var costValue = Math.round(globalObjective());
                consolePrint("Agent "+savedParameters[1]+" added, Global cost:"+costValue);
                if(savedParticles.length==savedParameters[1]){savedParameters[6].push(costValue);}
            }else{
                // end of the final centralized case
                submodularityMode = 1;
                var costArray = savedParameters[6];
                chosenMachineIndex = costArray.indexOf(Math.max.apply(Math,costArray));//this can be the result given by the centralized machine too
                print("Machine "+chosenMachineIndex+" Performed Best!")  
                if(!chosenMachineIndex==numOfMachines){//otherwise ok
                    //Have to reset agents to the positions given in : savedParameters[0][chosenMachineIndex]
                    particleShadows = [];particles = [];
                    for(var i=0;i<savedParameters[0][chosenMachineIndex].length;i++){
                        var pointInterested = submodularityCandidates[savedParameters[0][chosenMachineIndex][i]]; 
                        addAgentToPoint(pointInterested.x,pointInterested.y);
                        print("agents reloaded");
                    }
                }
                var m = Number(document.getElementById("mRandomizedGreedy").value);
                var approxFactor = Math.round(0.63212055882*1000/Math.min(m,savedParticles.length))/1000;
                if(approxFactor<0){approxFactor=0;}
                consolePrint("Solving Using Distributed - Randomized Greedy Algorithm: Finished after "+savedParameters[2]+" computations, with avg. approx. factor: "+approxFactor+".");
            }
            sleepFor(200);// for better display
            
        }
    }
    else if(submodularityMode==6){//simulate distributed - sequential greedy
        document.getElementById("displayCoverageDensity").checked = true;
        printPointArrayP2(savedParameters[3][savedParameters[1]],color(100,0,100),2);
        if(savedParticles.length>savedParameters[1]){// need to add an agent
            iterationOfDistributedSequentialGreedy(savedParameters[1]);// add the most benificial agent
            var costValue = Math.round(globalObjective());
            consolePrint("Agent "+savedParameters[1]+" added, Global cost: "+costValue+".");
            if(savedParticles.length==savedParameters[1]){savedParameters[6].push(costValue);}
        }else{
            submodularityMode = 1;
            var approxFactor = 0.5;
            if(approxFactor<0){approxFactor=0;}
            consolePrint("Solving Using Distributed - Sequential Greedy Algorithm: Finished after "+savedParameters[2]+" computations, with approx. factor: "+approxFactor+".");
        }
        sleepFor(200);// for better display
    }


    // end - submodularity display candidates
    /////////////////////////////////////////////



    // Q investigation: Use deltamat to get Q_i(k)
    //print(DeltaValuesMatrix);
    if(variableStepSizeMode && saveDataMode){
        var Q_iArray = [];
        for (var i = 0; i < particleShadows.length; i++) {
            
            var neighbors = particleShadows[i].getEffectiveNeighbors();
            var Q_i = 0;

            for(var k = 0; k < neighbors.length; k++){
                var j = neighbors[k];
                Q_i = Q_i + DeltaValuesMatrix[j][i] + DeltaValuesMatrix[j][j];

                var neighborsJ = particleShadows[j].getEffectiveNeighbors();
                for(var m = 0; m<neighborsJ.length; m++){
                    var l = neighborsJ[m]; 
                    if(l!==i){
                        Q_i = Q_i + DeltaValuesMatrix[l][j];
                    }
                }
            }
            QValues.push(Q_i);
            // curl Q_i ready
            //print("Q_"+i+" = "+Q_i);
            
            // uncomment following to save T values
            /*Q_iArray.push(Q_i);

            if(typeof Q_iSum[i] !== 'undefined'){
                if((Q_iSum[i] + Q_i)>=0){
                    Q_iSum[i] = 0;
                    if(typeof TValuesArray[i] !== 'undefined'){
                        TValuesArray[i][TValuesArray[i].length] = 0;// add new value
                    }else{
                        TValuesArray[i] = [0];
                    }
                    
                }else{
                    Q_iSum[i] = Q_iSum[i] + Q_i; 
                    
                    if(typeof TValuesArray[i] !== 'undefined'){
                        TValuesArray[i][TValuesArray[i].length-1] =  TValuesArray[i][TValuesArray[i].length-1]+1; //increment T
                    }else{
                        TValuesArray[i] = [1];
                    }

                }
                
            }
            else{
                if(Q_i>0){
                    Q_iSum[i] = 0;
                    TValuesArray[i][TValuesArray[i].length] = 0;// add new value
                }else{
                    Q_iSum[i] = Q_i; 
                    
                    if(typeof TValuesArray[i] !== 'undefined'){
                        TValuesArray[i][TValuesArray[i].length-1] =  TValuesArray[i][TValuesArray[i].length-1] + 1; //increment T
                    }else{
                        TValuesArray[i] = [1];
                    }

                }
                Q_iSum[i] = Q_i; 
                TValuesArray[i] = [1];
            }*/
            
        }
    }    
    ////print(Q_iSum);
    //QiSum =  QiSum + Q_i
    // end Q investigation



    //print(millis()-timeStart);
    


    // debug mode
    if(isDebugMode){

        // temporary2 - for move gradually and observe the gradient
        // var a = new Point2(90,10);
        // var b = new Point2(160,10);
        
        // var a = new Point2(180,180);
        // var b = new Point2(180,420);
        
        // var a = new Point2(240,260);
        // var b = new Point2(340,360);
        
        // particleShadows[1].position = new Point2(200,300)
        // var a = new Point2(300,300);
        // var b = new Point2(450,300);

        // var a = new Point2(200,300);
        // var b = new Point2(550,300);

        // var a = new Point2(249,251);
        // var b = new Point2(349,351);
        
        // moveGraduallyandObserve(0,a,b);
        // isDebugMode = false;
        // end temporary1
        
        // uncomment once done
        for (var i = 0; i < particleShadows.length; i++) {  
            
            var di = particleShadows[i].getDerivativesWithBoosting();
            var di = new Point2(di[0],di[1]);
             
            var sumdij = di; 

            var neighbors = particleShadows[i].getEffectiveNeighbors();
            for(var j = 0; j < neighbors.length; j++){
                
                var dij = particleShadows[i].getJointDerivative(neighbors[j],0);
                var dij = new Point2(dij[0],dij[1]);
                
                sumdij = plusP2(sumdij,dij);

                var x1 = particleShadows[i].position;
                var x2 = plusP2(x1,productP2(dij,2));
                var betaHatji = 2*(dotP2(di,dij))/dotP2(di,di);
                betaHatji = round(100*betaHatji)/100;
                drawArrow(x1,x2,5,"d"+(i+1).toString()+","+(neighbors[j]+1).toString()+"; b"+(neighbors[j]+1).toString()+","+(i+1).toString()+"= "+betaHatji.toString(),5);
                
            }

            var betaiStar = dotP2(di,sumdij)/((neighbors.length+1)*dotP2(di,di));
            betaiStar = round(100*betaiStar)/100;

            var x2 = plusP2(particleShadows[i].position,productP2(di,4));
            drawArrow(particleShadows[i].position,x2,5,"d"+(i+1).toString()+"; b*"+(i+1).toString()+"= "+betaiStar.toString(),5);
            

            // arc sections
            var result = particleShadows[i].getArcSections();
            var arcAnglePairs = result[0];//[[start,end],angle,point]
            var arcIntersectingPoints = result[2];
            for(var j =0; j<arcIntersectingPoints.length; j++){          
                ellipse(arcIntersectingPoints[j].x, arcIntersectingPoints[j].y, 10, 10);
            }

            
        }
    }





    
    //updating labels
    
    ////Objective Value

    objectiveValueNew = round(100*globalObjective())/100;
    
    coverageLevelDisplay=document.getElementById("objectiveDisplay");

    coverageLevelDisplay.innerHTML=int(objectiveValueNew);
    
    
    //// Number of Agents
    numberOfAgentsDisplay = document.getElementById("numberOfAgents");

    numberOfAgentsDisplay.innerHTML=particleShadows.length;
    
    
    
    //moving average (of coverage cost increment) update
    // counting with physical agents function needs this:
    movingAverage = movingAverage-(objectiveValueArray[objectiveValueArray.length-movingAverageWindowSize]-(objectiveValueNew-objectiveValue))/movingAverageWindowSize;
    
    iterationNumber++;



    // Periodic functions !!!
    // need to run the : testAgentLocalMinimaReached() or  testLocalMinimaReached() according to their period
    if(testAgentLocalMinimaReachedInterval>0){// decentralized boosting
        if(stateUpdateIterationNumber%testAgentLocalMinimaReachedInterval==0){
            var globalOptimalityReached = true;
            for(var i = 0; i<particleShadows.length; i++){
                if(particleShadows[i].isBoostingActivated!=9){
                    globalOptimalityReached = false;
                }
            }
            if(globalOptimalityReached){
                // pause
                stateUpdateIterationNumber++;
                stopParticle();
                executionTime = Math.round(millis()-executionTime)/1000;

                // end pause
                consolePrint("Global optimality: ~"+objectiveValue+" reached after "+executionTime+" s");
            }
            //////testAgentLocalMinimaReached();
        }
    }else if(testLocalMinimaReachedInterval>0){// centralized boosting
        if(stateUpdateIterationNumber%testLocalMinimaReachedInterval==0){
            testLocalMinimaReached();
        }
    }

    if(updateLocalLipschitzConstantsInterval>0){
        if(stateUpdateIterationNumber%updateLocalLipschitzConstantsInterval==0){
            tuneLipschitzConstantsLocally();
        }
    }
    // end - periodic functions 


    // data storing for plotting
    if(plotDataMode){
        updateAndStoreDataForPlotting();
        var datas = localStorage.getItem('switchKey1');
        var data = JSON.parse(datas);
        //print(data);
        if(data){
            consolePrint('Updating the Global Objective Function Plot...');
            calculateGlobalObjectiveValuesWRTs_N();
            data = false;
            datas = JSON.stringify(data);
            localStorage.setItem('switchKey1', datas);
            consolePrint('Updating finished.');        
        }
    }

   
    objectiveValue = objectiveValueNew; 
    
    // for save Text
    if(saveDataMode && addObstacleMode==0 && !isDebugMode && isSimulationMode && !mouseIsPressed){
    	////textToSave = textToSave+objectiveValue+"\r\n";
	}

    
    //counting
    if(isSimulationMode==1 && displayPhysicalAgentsMode && boostingMethod==0){
	    countingWithPhysicalAgents();
    }else if(isSimulationMode==1 && boostingMethod>0){
    	//need to do something here....
    	//print(agentPositions.length)
    }
    

    ////PSO display
    // if(solutionOfPSO.length>0){
    //     showParticleOfPSO(solutionOfPSO);
    // }
    //// end PSO display

    
    if((iterationNumber-startButtonClickedIteration) > 1 && startButtonClicked){
        //print("here");
        start();   
    }
    
}

