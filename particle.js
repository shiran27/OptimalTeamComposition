function Particle(x, y) {

    this.position = new Point2(x,y);
    this.nextPositionToBe  = new Point2(x,y); // to do the state update once after all calculations
    
    //look at this concept
    //this.shadowPosition = new Point2(x,y);//created shadow
    this.shadowsFollowingMode = 0; 
    
    ////P3
    ////var weightPGD = -15000; // weight coefficients for the cost function sum(t), default -1e4 for n=3;
    this.t = 1;
    var tolence = 0.001;
    this.nextTtoBe = [];
    ////P3
    
    this.id = particleShadows.length;
    var r = 12; //radius of the particle's circle
    //    var sensR=200; //sensing range
    
    this.velocity = new Point2(0,0);
    
    this.s = [];
    this.v = [];
    this.sRef = [];//smae as referencePointTrajectory
    this.u = [];
    
    
    this.uMaxSq = 100*100; //U_Max^2
    this.vMaxSq = 1000*1000; //V_Max^2
    
    this.trajectory = [];//same as s
    this.timeInstant = 0;//simulation instant living
    
    this.targetReferencePoint;
    this.referencePointTrajectory = [];
    
    this.wayPoints = [];//wayPoints to track
     
    // steady state of individual particles
    this.isBoostingActivated = false
    this.optimalCoverageH1 = 0; // coverage at the begining
    this.optimalCoverageS1 = this.position;
    this.boostingIterationNumber = 0;
 
    
    this.neighborsOld = [];
    this.combinedCoverageOld = 0;

    this.coverageLevel = 0;
    
    // for V - Boosting
    this.nearbyNonReflexVerticesFound = []; // not being used now (meant for an update for V - boosting)
    // for VA boostinf
    this.recentLocalCoverageLevels = [-1,0,0]; // can use to compare the coverage levels
    this.verticesFoundSoFarRegistry = []; // [obstacleIndex,vertexIndex,trackedOrNot]
    // trackedOrNot= vertices went around so far
    this.targetVertexAnchor = -1; // this would be Point2 value if we have an attraction point
    this.targetVertexAnchorIndex = -1; // index number of the targetVertexAnchor in the registry 
    // end V and VA- Boosting
    



    this.variableStepSizeMode = true;
    this.variableStepSizeModeDisabled = false; // to temporarily disble step size mode
    this.diminishingStepSizeMode = true;

    this.diminishingStepSizeModeInitiated = false;//starts only in special cases
    this.diminishingStepSizeIterationNumber = 0;
    
    this.betaiStarLowCount = 0;
    this.lipschitzConstantK1 = lipschitzConstantK1;


    //to detect terminal oscillations and to recalculate lipschitz constant 
    this.lastPosition = new Point2(x,y);
    this.oscillatingCount = 0;
    this.oscillatingWidth = 0;

    this.pastTrackPoints = [this.position]; //array to keep tabs on how agent moved


    // to asses the convergence !
    this.derivativeSumValue = new Point2(0,0);
    this.derivativeAbsSquareSum = 0;
    this.stepSizeSquareSum = 0;
    this.convergenceTestPeriodLength = 0;
    this.lastPositionChecked = this.position;
    
    this.localMinimaReached = false;


    //// for heterogeneity
    this.senRange = 200;
    this.sensingDecayFactor = 0.012;
    this.sensingCapacity = 30000;// dummy value
    this.agentCostRatio = 1; // ratio between commertial cost and sensing capacity


    this.updateSensingCapacity = function(){
        var sensingCapacity =  (2*Math.PI/sq(this.sensingDecayFactor))*(1-(1+this.sensingDecayFactor*this.senRange)*Math.exp(-1*this.sensingDecayFactor*this.senRange));
        this.sensingCapacity = sensingCapacity;
        return sensingCapacity;
    }

    this.sensingCapacity = this.updateSensingCapacity();
    
  

    //generate point to track 
    this.generateReferencePointTrajectory = function(){
    	
    	
    	//var newTrajectorySegment = getInteriorTrajectoryPointsP2(this.position,this.wayPoints[0],10);
    	var newTrajectorySegment = repeatP2(this.wayPoints[0],5);
		
    	this.referencePointTrajectory = newTrajectorySegment;
		this.targetReferencePoint = this.wayPoints[0];
		this.driveSegmentStateFeedbackControl(newTrajectorySegment.length);
		
    	for(var i=1;i<this.wayPoints.length;i++){

        	//newTrajectorySegment = getInteriorTrajectoryPointsP2(this.wayPoints[i-1],this.wayPoints[i],10);
        	newTrajectorySegment = repeatP2(this.wayPoints[i-1],5);
    		this.referencePointTrajectory = concat(this.referencePointTrajectory,newTrajectorySegment);
    		this.targetReferencePoint = this.wayPoints[i];
    		this.driveSegmentStateFeedbackControl(newTrajectorySegment.length);
    	}
    	
    	//this.driveStateFeedbackControl();
    }
    
    this.driveSegmentStateFeedbackControl = function(segmentLength){
    	
    	var W = 0;//pole position
    	var Zp = 0.95;
    	
    	var K1 = sq(Zp);//controller gains
    	var K2 = -2*Zp*cos(W);
    	
    	var fac1 = -K2-3;
    	var fac2 = -(K1+K2+1)/timeStepSqHf;
    	var fac3 = -2*(K2+2)/timeStep;
    	
    	//h^2/2 = timeStepSqHf; h=timeStep
    	s = [];
    	v = [];
    	sRef = subset(this.referencePointTrajectory,this.referencePointTrajectory.length-segmentLength,segmentLength);
    	u = [];
    	
    	if(this.s.length>0){
    		append(s,this.s[this.s.length-1]);//s_0
    		append(v,this.v[this.v.length-1]);//v_0
    	}
    	else{
    		append(s,this.position);//s_0
    		append(v,this.velocity);//v_0
    	}
    	append(u,new Point2(0,0))//u_0
    	
    	
    	for(var k=0; k<(sRef.length-2); k++){
    		var term1 = plusP2( productP2(u[k],fac1) , productP2(s[k],fac2) );
    		var term2 = plusP2( productP2(sRef[k],K1) , plusP2( productP2(sRef[k+1],K2) , sRef[k+2]) );
    		var term3 = plusP2( productP2(v[k],fac3) , productP2(term2,(1/timeStepSqHf)));
    		
    		var uNew = plusP2(term1 , plusP2(term2,term3) );
    		uNew = saturateP2(uNew,this.uMaxSq);
    		append(u, uNew);
    		
    		append(s, plusP2( s[k] , plusP2( productP2(v[k],timeStep) , productP2(u[k],timeStepSqHf) ) )  );
    		
    		var vNew = plusP2( v[k] , productP2(u[k],timeStep) );
    		vNew = saturateP2(vNew,this.vMaxSq);
    		append(v, vNew );
			
    		//print(this.targetReferencePoint);
    		if( k==(sRef.length-3) && distP2(this.targetReferencePoint,s[k+1])>5 ){
    			//print('Extending');
    			append(sRef,this.targetReferencePoint);
    		}
    		else if(k==(sRef.length-3) ){
    			//print(normP2(v[k+1]));
    			//v[k+1]=new Point2(0,0);
    			break;
    		}
    		
    		
    	}
    	//this.controlInputArray = this.u;
    	this.s = concat(this.s , s);
    	this.v = concat(this.v , v);
    	this.u = concat(this.u , u);
    	this.sRef = concat(this.sRef , sRef);
    	this.trajectory = this.s;
    	
    }
    
    
    this.driveStateFeedbackControl = function(){
    	//controller gains
    	/*var W = PI/3;//pole position
    	var Zp = 0.95;
    	var K1 = sq(Zp);
    	var K2 = -2*Zp*cos(W);*/
    	
    	var Zp = 0.95//pole position
    	
    	var K1 = sq(Zp);
    	var K2 = -2*Zp;
    	
    	var fac1 = -K2-3;
    	var fac2 = -(K1+K2+1)/timeStepSqHf;
    	var fac3 = -2*(K2+2)/timeStep;
    	
    	//h^2/2 = timeStepSqHf; h=timeStep
    	
    	
    	this.sRef = this.referencePointTrajectory;
    	append(this.s,this.position);//s_0
    	append(this.v,this.velocity);//v_0
    	append(this.u,new Point2(0,0))//u_0
    	
    	
    	for(var k=0; k<(this.sRef.length-2); k++){
    		var term1 = plusP2( productP2(this.u[k],fac1) , productP2(this.s[k],fac2) );
    		var term2 = plusP2( productP2(this.sRef[k],K1) , plusP2( productP2(this.sRef[k+1],K2) , this.sRef[k+2]) );
    		var term3 = plusP2( productP2(this.v[k],fac3) , productP2(term2,(1/timeStepSqHf)));
    		
    		var uNew = plusP2(term1 , plusP2(term2,term3) );
    		uNew = saturateP2(uNew,this.uMaxSq);
    		append(this.u, uNew);
    		
    		append(this.s, plusP2( this.s[k] , plusP2( productP2(this.v[k],timeStep) , productP2(this.u[k],timeStepSqHf) ) )  );
    		
    		var vNew = plusP2( this.v[k] , productP2(this.u[k],timeStep) );
    		vNew = saturateP2(vNew,this.vMaxSq);
    		append(this.v, vNew );
			
    		if( k==(this.sRef.length-3) && distP2(this.targetReferencePoint,this.s[k])>5 ){
    			//print('Extending');
    			append(this.sRef,this.targetReferencePoint)
    		}
    		else if(k==(this.sRef.length-3)){
    			break;
    		}
    		
    	}
    	//this.controlInputArray = this.u;
    	this.trajectory = this.s;
    	
    }
    
    
    
    this.update = function(particleNum) {//old update function
        this.id=particleNum;
        var v=[];
        v=speed(this.id);
            
        nextPosition = new  Point2(this.position.x+hStep*linearSpeed*v[0],this.position.y+hStep*linearSpeed*v[1]);    
        
        //adjustedNextPosition = this.avoidObstacles(nextPosition);
        
        this.position.x = nextPosition.x;
        this.position.y = nextPosition.y;
                            
        //moving in to use point2 arrays
        this.position = nextPosition;
        
    }
    
    this.quickUpdate = function(factor){// for the Greedy candidate update
        ////var derivatives = this.getDerivatives();
        var derivatives = this.getDerivativesWithoutt();

        // step size reduced by a factor of 5
        var nextPosition = new  Point2(this.position.x+constantStepSize*derivatives[0]/factor,this.position.y+constantStepSize*derivatives[1]/factor);    
        
        nextPosition = this.avoidObstacles(nextPosition);

        if(isOutOfCanvas(nextPosition)){//||isOutOfCanvas(nextPosition2)){
            //print("screwed1");
            nextPosition = avoidEscapeP2(this.position);// projecting to boundary
        }else if(isColorEqualP2CRigour(nextPosition,obstacleColor)){
            //print("screwed2");
            nextPosition  = this.pullOutFromObstacle();   
        }
        
        this.lastPosition = this.position;
        this.nextPositionToBe = nextPosition;
    }


    this.updateNew = function(particleNum) {//new update function
        this.id = particleNum;
        //var timeStart2 = millis();

        //var derivatives = this.getDerivatives();
        var derivatives = this.getDerivativesWithBoosting();
        
        //print(millis()-timeStart2);
        

        // for assessing he convergence
        this.derivativeSumValue = plusP2(this.derivativeSumValue,new Point2(derivatives[0],derivatives[1]));
        this.derivativeAbsSquareSum = this.derivativeAbsSquareSum+sq(derivatives[0])+sq(derivatives[1]);
        this.stepSizeSquareSum = this.stepSizeSquareSum+sq(constantStepSize);
        // end assesing convergence

        // loading for plotting
        ////derivativeValues.push(round(100*sqrt(sq(derivatives[0])+sq(derivatives[1])))/100);
        derivativeValues.push(round(100*derivatives[0])/100);// only one component
        betaValues.push(constantStepSize);
        QValues.push(0);// N/A

        print(round(100*derivatives[0])/100,round(100*derivatives[1])/100);

        // for save Text
        if(saveDataMode){
            //var textPart = round(100*sqrt(sq(derivatives[0])+sq(derivatives[1])))/100; 
            var textPart = round(100*derivatives[0])/100; 
            textToSave = textToSave+textPart+",";
            var textPart = round(100*derivatives[1])/100; 
            textToSave = textToSave+textPart+",";
        }

        var nextPosition = new  Point2(this.position.x+constantStepSize*derivatives[0],this.position.y+constantStepSize*derivatives[1]);    
        
        nextPosition = this.avoidObstacles(nextPosition);

        if(isOutOfCanvas(nextPosition)){//||isOutOfCanvas(nextPosition2)){
            print("screwed1");
            nextPosition = avoidEscapeP2(this.position);// projecting to boundary
        }else if(isColorEqualP2CRigour(nextPosition,obstacleColor)){
            print("screwed2");
            nextPosition  = this.pullOutFromObstacle();   
        }
        
        //// oscillation detection
        if(distP2(this.lastPosition,nextPosition)<1){
            //print("Oscillating "+this.id+' count: '+this.oscillatingCount);
            this.oscillatingCount ++;
            this.oscillatingWidth = distP2(this.position,nextPosition);
        }else{// no oscillations !!!
            this.oscillatingCount = 0;
        }
        this.lastPosition = this.position;
        //// end - oscillation detection

        this.nextPositionToBe = nextPosition;

        ////P3
        //var tStepSize = 0.001; //0.00001 is good
        //var tStepSize = 0.000005; //good for maze
        var tStepSize = 0.0001;//good for rest
        //var tStepSize = 0.001;//trying for the blank
        
        // optimum tStepSize
        if(variableStepSizeMode){
            var di =  this.localObjectiveFunctionWitht();
            var sumdij = di;
            var neighbors = this.getEffectiveNeighbors();
            var sumK1 = 1*(neighbors.length+1);
            for(var i = 0; i<neighbors.length; i++){
                sumdij = sumdij + this.crossDerivativeWitht(neighbors[i]);
            }
            tStepSize = di*sumdij/(sumK1*sq(di));
        }

        if(tBoostMode==0){
            var nextT = this.t + tStepSize*derivatives[2];
            //var nextT = this.t + 10*tStepSize*derivatives[2]/(this.agentCostRatio*this.sensingCapacity*this.t);
        }else{

            ////var nextT = this.t - tStepSize*derivatives[2];
            var nextT = this.t - tStepSize*this.agentCostRatio*this.sensingCapacity;
        }
        //var nextT = this.t + constantStepSize*derivatives[2];
        if (nextT > 1){
            nextT = 1;
        } else if (nextT < tolence){
            nextT = tolence;
        }
        this.nextTToBe = nextT;
        ////P3
    }


    
    this.variableStepSizeUpdate = function(particleNum) {//new update function
        
        this.id = particleNum;

        var di = this.getDerivativesWithBoosting();
        var diNP2; //derivative without boosting
        var diP2 = new Point2(di[0],di[1]);

        var modeOfSelf = getModeOfAgents([this.id]);

        // optimum step size calculation
        var sumdij;
        
        if(modeOfSelf==0){//not in a boosting mode
            diNP2 = diP2;
            sumdij = productP2(diNP2,1); 
        }else{// in a boosting mode
            if(this.boostingIterationNumber>4){// this will give pretty small beta*
                var diN = this.getDerivatives();
                diNP2 = new Point2(diN[0],diN[1]);
            }else{
                diNP2 = diP2;
            }
            sumdij = productP2(diNP2,1); 
        }


        //var sumdij2 = productP2(sumdij,1.5);

        // //          
        var sumdijt = 0;

        var dijVals = []; //to store them for Q 
         
        var neighbors = this.getEffectiveNeighbors();
        // //var modeOfNeighbors = getModeOfAgents(neighbors);
        var sumOfLipschitzConstants = this.lipschitzConstantK1;

        for(var j = 0; j < neighbors.length; j++){
            
            // // var dij = this.getJointDerivative(neighbors[j],modeOfNeighbors[j]);
            var dij = this.getJointDerivative(neighbors[j],0);
            var dijP2 = new Point2(dij[0],dij[1]);

            dijVals.push(dijP2);
            sumdij = plusP2(sumdij,dijP2);
            sumdijt = sumdijt + dij[2];
            // taking lipschitz constant fromneighbors to be more precise
            sumOfLipschitzConstants = sumOfLipschitzConstants + particleShadows[neighbors[j]].lipschitzConstantK1;
        
        }           

        //// var betaiStar = dotP2(di,sumdij)/(hStep*lipschitzConstantK1*(neighbors.length+1)*dotP2(di,di));
        //////var betaiStar = dotP2(diP2,sumdij)/(this.lipschitzConstantK1*(neighbors.length+1)*dotP2(diP2,diP2));
        
        // P3
        ///////var betaiStar = dotP2(diP2,sumdij)/(sumOfLipschitzConstants*dotP2(diP2,diP2));
        var betaiStar = (dotP2(diP2,sumdij)+di[2]*sumdijt)/(sumOfLipschitzConstants*(dotP2(diP2,diP2) + di[2]*di[2]));
        // P3





        // var betaiStar = dotP2(diP2,sumdij)/(lipschitzConstantK1*(neighbors.length+1));
        
        //// print("1: "+betaiStar);
        


        // start addressing pathalogical case
        if(abs(betaiStar)<0.05){
            this.betaiStarLowCount++;
        }else{
            this.betaiStarLowCount = 0;
        }
        // this can even start when agent is in his last normal mode 
        if(this.betaiStarLowCount>5 && this.variableStepSizeMode && this.isBoostingActivated==9 ){//start dim steps
            print("Activated");
            this.variableStepSizeMode = false; //switching off betai*
        }else if(!this.variableStepSizeMode){//have to remain in this mode till 


            // correction 1 to the patological case (removing al non positive influential neighbors)
            sumOfLipschitzConstants = this.lipschitzConstantK1;
            sumdij = productP2(diNP2,1);
            for(var j = 0; j < neighbors.length; j++){
            
                var dij = this.getJointDerivative(neighbors[j],0);
                var dijP2 = new Point2(dij[0],dij[1]);
                if(dotP2(dijP2,diNP2)>0){
                    sumdij = plusP2(sumdij,dijP2);
                    sumOfLipschitzConstants = sumOfLipschitzConstants + particleShadows[neighbors[j]].lipschitzConstantK1;
                }

            }           
            betaiStar = dotP2(diP2,sumdij)/(sumOfLipschitzConstants*dotP2(diP2,diP2));
            // end correction 1 to the pathalogical case

            


            // correction 2 to the patological case (removing most negative influential neighbors)
            // sumOfLipschitzConstants = this.lipschitzConstantK1;
            // sumdij = productP2(diNP2,1);
            // var neglectedNeighbors = [];
            // var neglectedNeighborCost = [];
            // for(var j = 0; j < neighbors.length; j++){
            
            //     var dij = this.getJointDerivative(neighbors[j],0);
            //     var dijP2 = new Point2(dij[0],dij[1]);
            //     if(dotP2(dijP2,diNP2)>=0){
            //         sumdij = plusP2(sumdij,dijP2);
            //         sumOfLipschitzConstants = sumOfLipschitzConstants + particleShadows[neighbors[j]].lipschitzConstantK1;
            //     }else{
            //         neglectedNeighbors.push(neighbors[j]);
            //         neglectedNeighborCost.push(Math.round(dotP2(dijP2,diNP2)*1000)/1000);  
            //         sumdij = plusP2(sumdij,dijP2);
            //         sumOfLipschitzConstants = sumOfLipschitzConstants + particleShadows[neighbors[j]].lipschitzConstantK1;
            //     }

            // }

            // if(neglectedNeighbors.length>0){
            //     var removeIndex = neglectedNeighborCost.indexOf(Math.min(...neglectedNeighborCost));  
            //     removeIndex = neglectedNeighbors[removeIndex];
            //     var dij = this.getJointDerivative(removeIndex,0);
            //     var dijP2 = new Point2(-1*dij[0],-1*dij[1]);
            //     sumdij = plusP2(sumdij,dijP2);
            //     sumOfLipschitzConstants = sumOfLipschitzConstants - particleShadows[removeIndex].lipschitzConstantK1;
            // }
            // var betaiStar = dotP2(diP2,sumdij)/(sumOfLipschitzConstants*dotP2(diP2,diP2));
            // end correction 2 to the pathalogical case



            
        }
        // end addressing the pathological case







        // bounding step : 
        //this is the maximum distance that can travel along the descent direction
        var stepSizeBound = this.getDistanceBoundToAvoidVertices(diP2,betaiStar); 

        // betaiStar upper bound should be calculated
        var betaiStarBound = stepSizeBound/(diP2.lengthP2()*constantStepSize);
        if(betaiStar>0 && betaiStar>betaiStarBound){
            betaiStar = betaiStarBound;
        }else if(betaiStar<0 && Math.abs(betaiStar)>betaiStarBound){
            betaiStar = -betaiStarBound;
        }
        // end bounding step







        // diminishing stepss
        if(neighbors.length==0 && diP2.lengthP2()<0.01){
            //print("gu");
            betaiStar = 1/this.lipschitzConstantK1;
            this.diminishingStepSizeModeInitiated = false;
            this.diminishingStepSizeIterationNumber = 0;

        }else if(neighbors.length==0 && diminishingStepSizeMode && !this.diminishingStepSizeModeInitiated){
            print("Dim Steps Started For "+ this.id);
            this.diminishingStepSizeModeInitiated = true;
            this.diminishingStepSizeIterationNumber = 1;
        }else if(neighbors.length==0 && diminishingStepSizeMode && this.diminishingStepSizeIterationNumber){
            print("Dim Steps Continues For "+ this.id);
            this.diminishingStepSizeIterationNumber++;
        }else{
            this.diminishingStepSizeModeInitiated = false;
            this.diminishingStepSizeIterationNumber = 0;
        }



        if(isNaN(betaiStar)||(betaiStar==Infinity)||(betaiStar==null)){
            print("Trouble Begins!!!");
            if(neighbors.length==0){
                betaiStar = 1/this.lipschitzConstantK1;
            }else{
                betaiStar = 1;
            }
        }

        ////print("2: "+betaiStar);
        ////print("3: "+betaiStar*di[0]);

        
        // Storing \Delta_ij s - just to investigate
        if(saveDataMode){
            var firstTermInDelta = productP2(diP2,betaiStar);
            var secondTermInDelta = -0.5*this.lipschitzConstantK1*pow(betaiStar,2)*dotP2(diP2,diP2);
            DeltaValuesMatrix[particleNum][particleNum] = dotP2(firstTermInDelta,diNP2) + secondTermInDelta; // \Delta_ii
            for(var j = 0; j < neighbors.length; j++){
                DeltaValuesMatrix[particleNum][neighbors[j]] = dotP2(firstTermInDelta,dijVals[j]) + secondTermInDelta;
            }
        }
        // end storing \Delta_ij s




        //INTERESTING: 
        if(!negativeStepsAllowedMode && betaiStar<0){
            betaiStar=0;
        }
        
        // for assessing he convergence
        this.derivativeSumValue = plusP2(this.derivativeSumValue,diP2);
        this.derivativeAbsSquareSum = this.derivativeAbsSquareSum + sq(diP2.lengthP2());
        this.stepSizeSquareSum = this.stepSizeSquareSum + sq(betaiStar*constantStepSize);
        ////print(this.id+': '+this.derivativeAbsSquareSum+' , '+sq(descretizationLevel/2)/this.stepSizeSquareSum);
        // end assesing convergence



        //for plotting
        ////derivativeValues.push(round(100*diP2.lengthP2())/100);
        derivativeValues.push(round(100*diP2.x)/100);
        betaValues.push(round(100*betaiStar)/100);
        // end for plotting

        print(this.id+","+round(100*betaiStar)/100+","+round(100*diP2.x)/100+","+round(100*diP2.y)/100+";");
        if(abs(betaiStar)<0.1 && diP2.lengthP2()>0.1){
            //betaiStar = 1/this.lipschitzConstantK1;
        }
        //print(sqrt(sq(derivatives[0])+sq(derivatives[1])));

        // for save Text
        if(saveDataMode){
            //var textPart = round(100*sqrt(sq(derivatives[0])+sq(derivatives[1])))/100; 
            var textPart = round(100*di[0])/100; 
            textToSave = textToSave+textPart+",";
            var textPart = round(100*di[1])/100; 
            textToSave = textToSave+textPart+",";
        }
        ////print("4: "+betaiStar*di[0]);
        // //var nextPosition = new  Point2(this.position.x+betaiStar*hStep*derivatives[0],this.position.y+betaiStar*hStep*derivatives[1]);  
        
        if(!this.diminishingStepSizeModeInitiated){
            var nextPosition = new  Point2(this.position.x+constantStepSize*betaiStar*di[0],this.position.y+constantStepSize*betaiStar*di[1]); 
        }else{
            var stepSizeCoefficient  = diminishingStepSizeK0/pow(this.diminishingStepSizeIterationNumber,diminishingStepSizer)
            print("Dim Step Coef: "+ stepSizeCoefficient);
            var nextPosition = new  Point2(this.position.x+stepSizeCoefficient*di[0],this.position.y+stepSizeCoefficient*di[1]); 
        }

        ////print("4.1: "+nextPosition.x);
        ////print("4.2: "+nextPosition.y);
        
        nextPosition = this.avoidObstacles(nextPosition);

        ////print("4.3: "+nextPosition.x);
        ////print("4.4: "+nextPosition.y);

        //var nextPosition2 = plusP2(nextPosition,productP2(normalizeP2(minusP2(nextPosition,this.position)),3)) 
        if(isOutOfCanvas(nextPosition)){//||isOutOfCanvas(nextPosition2)){
            print("screwed1");
            nextPosition = avoidEscapeP2(this.position);// projecting to boundary
        }else if(isColorEqualP2CRigour(nextPosition,obstacleColor)){
            print("screwed2");
            ////print("d1");print(this.position);
            ////print("d2");print(nextPosition);
            nextPosition  = this.pullOutFromObstacle();   
            ////print("d3");print(nextPosition);
        }

        ////print("5: "+nextPosition.x);
        ////print("6: "+nextPosition.y);
        
        //// oscillation detection
        if(distP2(this.lastPosition,nextPosition)<1){
            //print("Oscillating "+this.id+' count: '+this.oscillatingCount);
            this.oscillatingCount ++;
            this.oscillatingWidth = distP2(this.position,nextPosition);
        }else{// no oscillations !!!
            this.oscillatingCount = 0;
        }
        this.lastPosition = this.position;
        //// end - oscillation detection
        
        //// update position
        this.nextPositionToBe = nextPosition;
        //// end - update position


        ////P3
        ////P3
        //var tStepSize = 0.001; //0.00001 is good
        //var tStepSize = 0.000005; //good for maze
        //////var tStepSize = 0.0001;//good for rest
        var tStepSize = betaiStar;//good for rest
        
        var nextT = this.t + tStepSize*di[2];

        //var nextT = this.t + constantStepSize*derivatives[2];
        if (nextT > 1){
            nextT = 1;
        } else if (nextT < tolence){
            nextT = tolence;
        }
        this.nextTToBe = nextT;
        ////P3
        // var nextT = this.t + stepSizeCoefficient*di[2];
        // if (nextT > 1){
        //     nextT = 1;
        // } else if (nextT < tolence){
        //     nextT = tolence;
        // }
        // this.nextTToBe = nextT;
        ////P3

    }

    this.executeUpdate = function(){

        if(distP2(this.position,this.nextPositionToBe)>150){
            //print('Large Step: Tuning K1');
            //this.tuneLipschitzConstant();
        }
        this.position = this.nextPositionToBe;

        ////P3
        this.t = this.nextTToBe;
        ////P3


        //// update track points
        if(distP2(this.position, this.pastTrackPoints[this.pastTrackPoints.length-1])>100){
            this.pastTrackPoints.push(this.position);
        }
        // if(boostingMethod==6){
        //     noStroke();
        //     fill(100);
        //     printPointArrayP2(this.pastTrackPoints,"green",2);
        // }
        //// end updating past track points

        //testConvergences
        if(testJustConvergenceInterval>0){// only when boostingMethod==0
            this.testConvergenceForNoBoosting();
        }else if(testAgentLocalMinimaReachedInterval>0){// for decentralized boosting)
            this.updateDecentralizedBoostingProcess();
        }
        //end test converegences
    }


    this.getDistanceBoundToAvoidVertices = function(descentDirection,betaiStar){//maximum distance can be travelled in the direction of d

        //var E = this.position;
        //temporary next point: if variable step size moode
        var nextPosition = new  Point2(this.position.x+constantStepSize*betaiStar*descentDirection.x, this.position.y+constantStepSize*betaiStar*descentDirection.y);  

        var distanceBound = 0;

        // need to select bound depending on the mode we are in
        // lets activate bounds only when variable step size mode is active
        if(!variableStepSizeMode){
            return 1000;
        }else if(descentDirection.lengthP2()<0.01){//very small derivative - no point in imposing a bound
            return 1000;
        }
        // in variable step size mode with non zero magnitude derivative we have to do following
        

        // got the following from getFirstIntersectingVertices(a,b) function
        // because we need to idetify the first obstacle or the boundary
        var firstCollidingEdge = getFirstIntersectingVertices(this.position,nextPosition);


        if(firstCollidingEdge.length!=2){
            //// print("FaultFisrtBound");//Fault1y Colliding Boundary!!!
            return 1000;
        }else{
            // calculate boud for beta depending on : a, b, firstCollidingEdge
            var intersectingPoint = getIntersection(this.position, nextPosition, firstCollidingEdge[0], firstCollidingEdge[1])
            
            // guard is 5 pixels
            var guard = 5;
            var result = distP2(intersectingPoint,this.position)
            if(result>guard){
                return (result-guard);
            }else{
                return 0;
            } 
        }


    }




    this.avoidObstacles = function(nextPosition){
    	
        var nextPosition1 = nextPosition;
        var nextPosition2 = nextPosition;

        // upper bound to the motion
        if(distP2(this.position,nextPosition)>=(this.senRange)){
            //print("slow");
            nextPosition1 = plusP2(this.position,productP2(normalizeP2(minusP2(nextPosition,this.position)),this.senRange));
        }

        if(distP2(nextPosition1,this.position)>0.01){
            nextPosition2 =  plusP2(nextPosition1,productP2(normalizeP2(minusP2(nextPosition1,this.position)),4));  
        }
        
        

    	var result = nextPosition1;

        if(isOutOfCanvas(nextPosition1)){
            ////print("Escaping "+this.id+"to"+round(nextPosition1.x),round(nextPosition1.y));

            // check whether he is trying to go way-way out (i.e. both coordinates violate)
            var val = isWayOutOfCanvas(nextPosition1)
            if(val[0]){
                // print("here 1");
                // is there obstacles in between???? 
                if(isLineOfSight(this.position,val[1])){
                    return val[1]; // go to the corner    
                }else{// there are obstacles in the middle !!!
                    // print("Jump1");//first colliding edge or boundry is found from:
                    var firstCollidingEdge = getFirstIntersectingVertices(this.position,nextPosition1);
                    if(firstCollidingEdge.length!=2){
                        print("Fault0");//Fault1y Colliding Boundary!!!
                        return avoidEscapeP2(this.position);//stay put
                    }else{
                        result = getProjection(this.position, firstCollidingEdge[0], firstCollidingEdge[1], nextPosition1)
                        return result;
                    }
                }


            }

            // only one colliding edge
            //var collidingBoundary = getIntersectingBoundaryEdges(this.position,nextPosition);
            var collidingBoundary = getFirstIntersectingVertices(this.position,nextPosition1);
            // print(collidingBoundary);

            if(collidingBoundary.length!=2){
                print("Fault1");//Fault1y Colliding Boundary!!!
                print(round(this.position.x),round(this.position.y));
                print(round(nextPosition1.x),round(nextPosition1.y));
                return avoidEscapeP2(this.position);//stay put
            }

            // we have a proper colliding boundary
            result = getProjection(this.position, collidingBoundary[0], collidingBoundary[1], nextPosition1); // E,A,B,C,D

            if(isLineOfSight(this.position,result)){
                //result is ok
                return result;
            }else{// have to do something
                ////print("Jump2")//jumping over obstacles
                //first colliding edge or boundry is found from:
                var firstCollidingEdge = getFirstIntersectingVertices(this.position,nextPosition1);//error same line
                if(firstCollidingEdge.length!=2){
                    print("Fault2");//Fault1y Colliding Boundary!!!
                    return avoidEscapeP2(this.position);//stay put
                }else{
                    result = getProjection(this.position, firstCollidingEdge[0], firstCollidingEdge[1], nextPosition1)
                    return result;
                }
            }
        

        }
        else if(!isLineOfSight(this.position,nextPosition1)){
            ////print("obstacle Ahead 1 :"+this.id);
            var firstCollidingEdge = getFirstIntersectingVertices(this.position,nextPosition1);
            if(firstCollidingEdge.length!=2){
                print("Fault4");//Fault1y Colliding Boundary!!!
                print(round(this.position.x),round(this.position.y));
                print(round(nextPosition1.x),round(nextPosition1.y));
                print(round(result.x),round(result.y));
                return avoidEscapeP2(this.position);//stay put

            }else{
                result = getProjection(this.position, firstCollidingEdge[0], firstCollidingEdge[1], nextPosition1);
                if(isColorEqualP2C(result,obstacleColor)){// projection went wrong !!!!
                    print("fault5");
                    print(round(this.position.x),round(this.position.y));
                    print(round(nextPosition1.x),round(nextPosition1.y));
                    print(round(result.x),round(result.y));
                }
                return result;
            }

        }else if((!isLineOfSight(this.position,nextPosition2))&&(distP2(nextPosition1,this.position)>0.01)){
            ////print("obstacle Ahead 2 :"+this.id);
            var firstCollidingEdge = getFirstIntersectingVertices(this.position,nextPosition2);
            if(firstCollidingEdge.length!=2){
                print("Fault6");//Fault1y Colliding Boundary!!!
                print(round(this.position.x),round(this.position.y));
                print(round(nextPosition2.x),round(nextPosition2.y));
                print(round(result.x),round(result.y));
                return avoidEscapeP2(nextPosition2);//avoidEscapeP2(this.position);//stay put
            }else{
                result = getProjection(this.position, firstCollidingEdge[0], firstCollidingEdge[1], nextPosition2);
                if(isColorEqualP2C(result,obstacleColor)){// projection went wrong !!!!
                    print("fault7");
                    print(round(this.position.x),round(this.position.y));
                    print(round(nextPosition2.x),round(nextPosition2.y));
                    print(round(result.x),round(result.y));
                }
                return result;
            }
        }
    	////print("Default :"+this.id);
        return result;
    	
    }
        
    this.pullOutFromObstacle = function(){
        var stepSize = 2;
        var senRangeRed = 30;
        
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        
        var derivativeXPart1 = 0;
        var derivativeYPart1 = 0; 
        
        //initialize variables with dummy values
        var interestedPoint = new Point2(0,0);  var eventDensity=0; var dist=0;
        var distX=0; var distY=0; var constantTermW1=0;

        
        //var timeStart3 = millis();

        /*for (var x = this.position.x-senRangeRed+halfStepSize; x<=this.position.x+senRangeRed-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRangeRed+halfStepSize; y<=this.position.y+senRangeRed-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                interestedPoint = new Point2(x,y);
                eventDensity = getEventDensity(interestedPoint);
                dist = distP2(this.position,interestedPoint);
                
                if( eventDensity>0 && dist<this.senRange){
                    //ellipse(x,y,2,2);
                    distX = this.position.x-interestedPoint.x;
                    distY = this.position.y-interestedPoint.y;
                    //if(dist==0||distX==0||distY==0){print("ohhh it is needed"+dist==0);}
                    constantTermW1 = -1*eventDensity*areaFactor;
                    
                    derivativeXPart1 = derivativeXPart1 + (constantTermW1)*distX/dist;
                    derivativeYPart1 = derivativeYPart1 + (constantTermW1)*distY/dist;
                       
                }
            }
        }
        var nextStepDirection = new Point2(derivativeXPart1,derivativeYPart1);
        var nextPosition = plusP2(this.position,productP2(normalizeP2(nextStepDirection),senRangeRed));   
        return nextPosition;

    } 



    // this function change the position of the particle to the mouse location if the particle is dragged
    this.clicked = function() {
    	
        var d = dist(mouseX, mouseY, this.position.x, this.position.y);
        if (d < r) {
            /*this.position.x = mouseX;
            this.position.y = mouseY;
            this.position = new Point2(this.position.x,this.position.y);*/
            return true;
        }
        else{
        	return false;
        }
        
    }



    this.show = function(particleNum) {
        noStroke();
        fill(65, 74, 244);
        ellipse(this.position.x, this.position.y, 2*r, 2*r);

        //stroke(0);
        fill(0,0,0);
        rectMode(CENTER);
        textAlign(CENTER,CENTER);
        text(particleNum+1,this.position.x,this.position.y,2*r,2*r);
        
        
        //trajectory printing
        if(this.trajectory.length>0){
        	printPointArrayP2(this.trajectory,"blue",1);
        }
        
        if(typeof this.targetReferencePoint != 'undefined'){
        	fill(0);
            //ellipse(this.targetReferencePoint.x, this.targetReferencePoint.y, r, r);
            printPointArrayP2(this.referencePointTrajectory,"red",1);
            printPointArrayP2(this.sRef,"green",3);
        }

    }
    
    
    this.showShadow = function(particleNum){
        if((decentralizedBoostingMethod==1 && boostingMethod>0 && this.isBoostingActivated==1)||(decentralizedBoostingMethod==0 && boostingMethod>0 && boostingActivated==1)){
            noStroke();//BM
            fill(0);
            ellipse(this.position.x, this.position.y, 3*r, 3*r);
            if(boostingMethod==7 && this.targetVertexAnchor!==-1){// VA boosting target  VA point print
                //noStroke();
                fill(255,0,0);
                ellipse(this.targetVertexAnchor.x, this.targetVertexAnchor.y, 0.5*r, 0.5*r);
            }
        }else if((decentralizedBoostingMethod==1 && boostingMethod>0 && this.isBoostingActivated==2)||(decentralizedBoostingMethod==0 && boostingMethod>0 && boostingActivated==2)){
            noStroke();//NM1
            fill(0,0,255);
            ellipse(this.position.x, this.position.y, 3*r, 3*r);
            
        }else if((decentralizedBoostingMethod==1 && boostingMethod>0 && this.isBoostingActivated==9)||(decentralizedBoostingMethod==0 && boostingMethod>0 && boostingActivated==3)){
            noStroke();// finished !
            fill(0,255,0);
            ellipse(this.position.x, this.position.y, 3*r, 3*r);
        }else if(decentralizedBoostingMethod==1 && boostingMethod>0 && this.isBoostingActivated>=3 && this.isBoostingActivated<9){
            noStroke();//NM2
            fill(0,255,255);
            ellipse(this.position.x, this.position.y, 3*r, 3*r);
        }//otherwise NM0-no rings
        ////else if(boostingMethod==0 && this.localMinimaReached){
        else if(boostingMethod==0){
            
            /*if (this.t >= 0.05){
                noStroke();// finished !
                fill(0,0,255);
                ellipse(this.position.x, this.position.y, 3*r, 3*r);
            }else{
                noStroke();// finished !
                fill(0,0,0);
                ellipse(this.position.x, this.position.y, 3*r, 3*r); 
            }*/

        }


        if(submodularityMode!=2){
            noStroke();
            fill(244, 66, 66, this.t+0.5);
            ellipse(this.position.x, this.position.y, 2.5*r, 2.5*r);

            //stroke(0);
            fill(0,0,0);
            rectMode(CENTER);
            textAlign(CENTER,CENTER);
            text(this.id+1,this.position.x,this.position.y,2*r,2*r);
            if(this.t!=1){
                text("t="+Math.round(this.t*100)/100,this.position.x+20,this.position.y-20,2*r,2*r);
            }
        }else{
            noStroke();
            fill(100, 0, 100);
            ellipse(this.position.x, this.position.y, 1*r, 1*r);
        }
        
        
        
        //trajectory printing
        if(this.trajectory.length>0){
        	printPointArrayP2(this.trajectory,"blue",4);
        }
        
        if(typeof this.targetReferencePoint != 'undefined'){
        	fill(0);
            //ellipse(this.targetReferencePoint.x, this.targetReferencePoint.y, r, r);
            printPointArrayP2(this.referencePointTrajectory,"red",1);
            printPointArrayP2(this.sRef,"green",3);
        }

   
    }

          
    this.neighbor = function() {//old code 
        var neighbors=[];
        for (var i = 0; i < particles.length; i++) {
            if(i!=this.id){
            	if (distP2(this.position,particles[i].position)<(this.senRange+particleShadows[i].senRange)){
                    neighbors.push(i);
                }
            }
        }
        return neighbors;
    }
    
    this.updateDebug = function(){
    	this.position.x = mouseX;
    	this.position.y = mouseY;
    }
    
    this.getNearbyNonReflexVertices = function(){
        
    	var nearbyNonReflexVertices = [];
        for (var i = 0; i < obstacles.length; i++) { //search all obstacle non reflex vertices
        	for (var j = 0; j < obstacles[i].nonReflexVertices.length; j++){
        		var s = this.position;
        		var v_ij = obstacles[i].nonReflexVertices[j];
        		if(distP2(s,v_ij) < this.senRange){
        			//check intersectin with obstacle
        			//two pixels before and after (very close) the intersection point

                    if(checkNearPixels(s,v_ij,2)){
                        
                        //result is array of pairs [obstacle index,NRV index]
                        nearbyNonReflexVertices.push(i);//obstacle index
                        nearbyNonReflexVertices.push(j);//NRV index
                        //need to get to z corresponding to s and v_ij
                    }
       			
        		}
        	}
        }
        //print(nearbyNonReflexVertices);
        //this.nearbyNonReflexVertices = nearbyNonReflexVertices;
        return nearbyNonReflexVertices;
  
    }
    
    
    
    //sensing using sensingDecayFactor by slider
    this.sensingModelFunction = function(interestedPoint) {
    	var p;
    	var dis = distP2(this.position,interestedPoint);
    	
    	if (dis > this.senRange) {//out of range
    	    p = 0;
    	}
    	else if(isLineOfSight(this.position,interestedPoint)==false){
    		p = 0; //not in line of sight
    		
    	}
    	else{
    	    p = this.t*Math.exp(-this.sensingDecayFactor*dis);
    		//print("this is it");
    	
    	}
    	return p; 
    }
    

    this.sensingModelFunctionWithoutt = function(interestedPoint) {
        var p;
        var dis = distP2(this.position,interestedPoint);
        
        if (dis > this.senRange) {//out of range
            p = 0;
        }
        else if(isLineOfSight(this.position,interestedPoint)==false){
            p = 0; //not in line of sight
            
        }
        else{
            p = Math.exp(-this.sensingDecayFactor*dis);
            //print("this is it");
        
        }
        return p; 
    }


        //sensing using sensingDecayFactor by slider
    this.sensingModelFunctionForARay = function(interestedPoint) {
        var p;
        var dis = distP2(this.position,interestedPoint);
        
        if (dis > this.senRange) {//out of range
            p = 0;
        }
        else{
            p = Math.exp(-this.sensingDecayFactor*dis);
            //print("this is it");
        
        }
        return p; 
        
    }


    this.getEffectiveNeighbors = function(){

        var stepSize;
        if(obstacles.length>0){
            var stepSize = descretizationLevel;
        }else{
            var stepSize = descretizationLevel;
        }

        var tentativeNeighbors = this.getNeighbors();
        var sensitiveRange = 2*stepSize; //overlapping atleast 2 grid points
        var commonPointsThreshold = 4; // when obstacles are there this should b minimum common points
        var halfStepSize = stepSize/2;
        var effectiveNeighbors = [];
        
        
        for(var j1 = 0; j1<tentativeNeighbors.length; j1++){
            var j = tentativeNeighbors[j1];
            // sensitiveRange < 2R-d
            if(((this.senRange+particleShadows[j].senRange) - distP2(this.position,particleShadows[j].position))>=sensitiveRange){
                effectiveNeighbors.push(j);
            }
        }
        

        // when there are no obstacles
       // no isolations possible
        if(obstacles.length==0){
            return effectiveNeighbors;
        }else if(effectiveNeighbors.length==0){
            return []; //no need to further investigate each left
        }

        
        //when obstacles are there:
        tentativeNeighbors = effectiveNeighbors;
        effectiveNeighbors = [];

        var interestedPoint = new Point2(0,0);  var eventDensity=0; var disti=0; var distj=0;

        // i refers to this. (i = this.id);
        for (var j1 = 0; j1 < tentativeNeighbors.length; j1++) {

            var j = tentativeNeighbors[j1]; // neighbor id

            var commonPointsCount = 0;
            var foundEnoughPoints = false;
            /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
                for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+this.senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                    //fill(0);
                    //ellipse(x,y,2,2);
                    interestedPoint = new Point2(x,y);

                    // does this point is in the intersection?
                    eventDensity = getEventDensity(interestedPoint);
                    disti = distP2(this.position,interestedPoint);
                    distj = distP2(particleShadows[j].position,interestedPoint);
                    
                    if(eventDensity > 0 && disti < this.senRange && distj < this.senRange){//intersecting point!!!
                        if(isLineOfSight(this.position,interestedPoint)&&isLineOfSight(particleShadows[j].position,interestedPoint)){
                            commonPointsCount++;
                            if(commonPointsCount==commonPointsThreshold){
                                foundEnoughPoints = true;
                                break;
                            }   
                        }
                    }
                }
                
                if(foundEnoughPoints){
                    break;
                }
            }

            if(foundEnoughPoints){
                effectiveNeighbors.push(j);
            }

        }
        return effectiveNeighbors;
    }
       
    this.getNeighbors = function(){
        var neighbors=[];
        for (var i = 0; i < particleShadows.length; i++) {
        	if(particleShadows[i].id!=this.id){
        		if (distP2(this.position,particleShadows[i].position)<(this.senRange + particleShadows[i].senRange)){
        			neighbors.push(i);//this does not include the current (this.)
        		}
        	}
        }
        return neighbors;
    }
    
    //This function calculates the joint probability of detection 
    //at a given point with the help of the neightbours
    this.detectionProbabilityWithoutt = function(interestedPoint,exclusiveness){//exahustive method
        var neighbours = this.getNeighbors();
        var jointMissProbability = 1;
        for (var i = 0; i < neighbours.length; i++){
        	jointMissProbability = jointMissProbability*(1-particleShadows[neighbours[i]].sensingModelFunctionWithoutt(interestedPoint));
        }
        if(exclusiveness){
        	return (1-jointMissProbability);
        }
        else{//detection by itself
        	jointMissProbability = jointMissProbability*(1-this.sensingModelFunctionWithoutt(interestedPoint));
        	return (1-jointMissProbability);
        }
    }

    this.detectionProbability = function(interestedPoint,exclusiveness){//exahustive method
        var neighbours = this.getNeighbors();
        var jointMissProbability = 1;
        for (var i = 0; i < neighbours.length; i++){
            jointMissProbability = jointMissProbability*(1-particleShadows[neighbours[i]].sensingModelFunction(interestedPoint));
        }
        if(exclusiveness){
            return (1-jointMissProbability);
        }
        else{//detection by itself
            jointMissProbability = jointMissProbability*(1-this.sensingModelFunction(interestedPoint));
            return (1-jointMissProbability);
        }
    }

    this.detectionProbabilityWRTNeighbors = function(interestedPoint,exclusiveness,neighbours){//exahustive method
        var jointMissProbability = 1;
        for (var i = 0; i < neighbours.length; i++){
            jointMissProbability = jointMissProbability*(1-particleShadows[neighbours[i]].sensingModelFunction(interestedPoint));
        }
        if(exclusiveness){
            return (1-jointMissProbability);
        }
        else{//detection by itself
            jointMissProbability = jointMissProbability*(1-this.sensingModelFunction(interestedPoint));
            return (1-jointMissProbability);
        }
    }

    this.detectionProbabilityWRTNeighborsWithoutt = function(interestedPoint,exclusiveness,neighbours){//exahustive method
        var jointMissProbability = 1;
        for (var i = 0; i < neighbours.length; i++){
            jointMissProbability = jointMissProbability*(1-particleShadows[neighbours[i]].sensingModelFunctionWithoutt(interestedPoint));
        }
        if(exclusiveness){
            return (1-jointMissProbability);
        }
        else{//detection by itself
            jointMissProbability = jointMissProbability*(1-this.sensingModelFunctionWithoutt(interestedPoint));
            return (1-jointMissProbability);
        }
    }
    
    this.detectionProbabilityWithout = function(interestedPoint,exclusiveness,neighborAgent){//exahustive method
        var neighbours = this.getNeighbors();
        var jointMissProbability = 1;
        for (var i = 0; i < neighbours.length; i++){
            if(neighbours[i]!=neighborAgent){
                jointMissProbability = jointMissProbability*(1-particleShadows[neighbours[i]].sensingModelFunction(interestedPoint));
            }
        }
        if(exclusiveness){
            return (1-jointMissProbability);
        }
        else{//detection by itself
            jointMissProbability = jointMissProbability*(1-this.sensingModelFunction(interestedPoint));
            return (1-jointMissProbability);
        }
    }


    this.objectiveFunction = function(){//local yet exaustive
    	var stepSize = 10; 
    	var areaFactor = sq(stepSize);
    	var halfStepSize = stepSize/2;
    	var objectiveValueLocal = 0;
    	   	
    		
    	/*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
            	//ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if(eventDensity>0 && dist<this.senRange){
                	//ellipse(x,y,2,2);
                	objectiveValueLocal = objectiveValueLocal + this.detectionProbability(interestedPoint,false)*eventDensity*areaFactor;
                }
            }
        }
        //print(objectiveValue);
    	return objectiveValueLocal;	
    }

    
    this.localObjectiveFunction = function(){//local yet exaustive
        var stepSize = descretizationLevel; 
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        var objectiveValueLocal = 0;
            
            
        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if(eventDensity>0 && dist<this.senRange){
                    //ellipse(x,y,2,2);
                    ////objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbability(interestedPoint,true))*this.sensingModelFunction(interestedPoint)*eventDensity*areaFactor;
                    objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbabilityWithoutt(interestedPoint,true))*this.sensingModelFunctionWithoutt(interestedPoint)*eventDensity*areaFactor;
                }
            }
        }
        //print(objectiveValue);
        return objectiveValueLocal; 
    }


    // P3
    this.localObjectiveFunctionWitht = function(){// derivative with respect to t_i of H(s,t) = H_i(\bar{s}_i,\bar{t}_i^c)
        var stepSize = descretizationLevel; 
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        var objectiveValueLocal = 0;
              
        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if(eventDensity>0 && dist<this.senRange){
                    //ellipse(x,y,2,2);
                    ////objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbability(interestedPoint,true))*this.sensingModelFunction(interestedPoint)*eventDensity*areaFactor;
                    objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbability(interestedPoint,true))*this.sensingModelFunctionWithoutt(interestedPoint)*eventDensity*areaFactor;
                }
            }
        }
        //print(objectiveValue);

        return objectiveValueLocal - weightPGD*this.sensingCapacity*this.agentCostRatio; 
    }

    this.crossDerivativeWitht = function(j){// d_ij 

        var stepSize = descretizationLevel; 
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        var objectiveValueLocal = 0;
              

        var neighbors = this.getEffectiveNeighbors();
        
        var index = neighbors.indexOf(j);
        if (index > -1) {
          neighbors.splice(index, 1);
        }

        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist1 = distP2(this.position,interestedPoint);
                var dist2 = distP2(particleShadows[j].position,interestedPoint);


                if(eventDensity>0 && dist1 < this.senRange && dist2 < particleShadows[j].senRange){
                    //ellipse(x,y,2,2);
                    ////objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbability(interestedPoint,true))*this.sensingModelFunction(interestedPoint)*eventDensity*areaFactor;
                    objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbabilityWRTNeighbors(interestedPoint,true,neighbors))*this.sensingModelFunctionWithoutt(interestedPoint)*particleShadows[j].sensingModelFunctionWithoutt(interestedPoint)*eventDensity*areaFactor;
                }
            }
        }
        //print(objectiveValue);

        return -1*particleShadows[j].t*objectiveValueLocal; 
    }

    // end P3

    this.localObjectiveFunctionWRTNeighbors = function(setOfNeighbors){
        var stepSize = descretizationLevel; 
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        var objectiveValueLocal = 0;
            
        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if(eventDensity>0 && dist<this.senRange){
                    //ellipse(x,y,2,2);
                    objectiveValueLocal = objectiveValueLocal + (1-this.detectionProbabilityWRTNeighborsWithoutt(interestedPoint,true,setOfNeighbors))*this.sensingModelFunctionWithoutt(interestedPoint)*eventDensity*areaFactor;
                }
            }
        }
        //print(objectiveValue);
        return objectiveValueLocal; 
    }


    // this is not exactly correct (but roughly it is okay) lets replace it by this.getLocalCombinedCoverageLevel()
    this.getLocalCoverageLevel = function(){

        var neighbours = this.getEffectiveNeighbors();
        var localCoverageLevel = 0;

        for (var i = 0; i < neighbours.length; i++){
            localCoverageLevel = localCoverageLevel + particleShadows[neighbours[i]].coverageLevel;
        }
        return localCoverageLevel + this.coverageLevel;
    }

    this.getLocalCombinedCoverageLevel = function(){
        var neighbors = this.getEffectiveNeighbors();
        var jointAgents = neighbors;
        var disjointAgents = [];
        var localCombinedCoverageLevel = this.getCustomLocalCoverageLevel(jointAgents,disjointAgents);
        
        disjointAgents.push(this.id);
        jointAgents.shift();

        for (var i = 0; i < neighbors.length; i++) {
            localCombinedCoverageLevel = localCombinedCoverageLevel + particleShadows[neighbors[i]].getCustomLocalCoverageLevel(jointAgents,disjointAgents);
            disjointAgents.push(neighbors[i]);
            jointAgents.shift();
        }
        return localCombinedCoverageLevel;   
    }

    this.getCustomLocalCoverageLevel = function(jointAgents, disjointAgents){
        // coverage of "this." with the contributions coming from jointAgents but totally neglecting areas correspond to disjoint agents
        // input arguements can be empty matrices
        // contributions from any other agent except for [this., jointAgents] will be disregarded here
        var stepSize = descretizationLevel; 
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        var customLocalCoverageLevel = 0;
            
            
        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if(eventDensity>0 && dist<this.senRange){
                    //ellipse(x,y,2,2);

                    // see whether this point can be sensed by atleast one of the agents in disjoint set
                    var notSensedByDisjointAgents = true; //if sensed, it will be zero
                    for (var i = 0; i<disjointAgents.length; i++){
                        notSensedByDisjointAgents = notSensedByDisjointAgents*(particleShadows[disjointAgents[i]].sensingModelFunction(interestedPoint)==0);
                    }

                    if(notSensedByDisjointAgents){

                        var jointMissProbability = 1;
                        for (var i = 0; i < jointAgents.length; i++){
                            jointMissProbability = jointMissProbability*(1-particleShadows[jointAgents[i]].sensingModelFunction(interestedPoint));
                        }
                        jointMissProbability = jointMissProbability*(1-this.sensingModelFunction(interestedPoint));
                        customLocalCoverageLevel = customLocalCoverageLevel + (1-jointMissProbability)*eventDensity*areaFactor;

                    }else{
                        // no increment needed for objectiveValueLocal
                    }
                    
                }
            }
        }
        //print(objectiveValue);
        return customLocalCoverageLevel; 
    }




    this.getDerivatives = function(){//local yet exaustive maybe

    	var stepSize = 10;
        if(descretizationLevel!=null && descretizationLevel!=NaN){
            stepSize = descretizationLevel;
        }else{
            stepSize = 10;
        }
    	var areaFactor = sq(stepSize);
    	var halfStepSize = stepSize/2;
    	
    	var derivativeXPart1 = 0; var derivativeXPart2 = 0;
    	var derivativeYPart1 = 0; var derivativeYPart2 = 0;
    	
    	//var timeStart3 = millis();
    	/*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){
                */
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        //print(p);
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
            	//ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if( eventDensity>0 && dist<this.senRange && dist!=0){
                	//ellipse(x,y,2,2);
                	var distX = this.position.x-interestedPoint.x;
                	var distY = this.position.y-interestedPoint.y;
                	
                	if(dist!=0){//otherwise not defined
                		var constantTerm = eventDensity*(1-this.detectionProbability(interestedPoint,true))*(-1*this.sensingDecayFactor)*this.sensingModelFunction(interestedPoint)*areaFactor/dist;
                		if (distX!=0){
                    		derivativeXPart1 = derivativeXPart1 + constantTerm*distX;
                    	}
                    	if (distY!=0){
                    		derivativeYPart1 = derivativeYPart1 + constantTerm*distY;
                    	}
                	}
                	
                }
            }
        }
    	//print("P1: "+Math.round(millis()-timeStart3));
    	//var timeStart4 = millis();
    	

        //part2
        stepSize = stepSize/5;//for more resolution

        ////
        var s; var v_j; var n_j; var D_j; var pointArray;
    	var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();
    	if(nearbyNonReflexVertices.length==0){
    		derivativeXPart2 = 0;
            derivativeYPart2 = 0;
    	}
    	else{//need to do the summation
    		for(var j = 0; j < nearbyNonReflexVertices.length; j = j+2){
    			s = this.position;
    			v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
    			
    			var D_j = distP2(s,v_j);
    			
    			n_j = normalizeP2(new Point2(-(s.y-v_j.y),(s.x-v_j.x)));
    			if(!isLineOfSight(s,plusP2(intermediateP2(s,v_j,-0.3),productP2(n_j,2)))){
    				n_j = productP2(n_j,-1);
    			}//now take sign(nj.x)
    			
    			//sin(theta_j)/D_j; theta_j between [0,pi/2];
    			var factor1 = Math.sign(n_j.x)*abs(v_j.y-s.y)/sq(D_j);
    			var factor2 = Math.sign(n_j.y)*abs(v_j.x-s.x)/sq(D_j);
    			
    			
    			
    			pointArray = pointsBeyondP2(this,v_j,stepSize);
    			pointArray.pop();

                //fill(120);
                //print(pointArray);
                //printPointArrayP2(pointArray,"red",2);
    			
                var sum1 = 0;
    			for(var i =0; i<pointArray.length; i++){
    				var x = pointArray[i];//phro_j(r)
    				var r = distP2(x,v_j);
    				
    				sum1 = sum1 + getEventDensity(x)*(1-this.detectionProbability(x,true))*this.sensingModelFunction(x)*r*stepSize; 
    			}
    			
    			derivativeXPart2 = derivativeXPart2 + factor1*sum1;
    			derivativeYPart2 = derivativeYPart2 + factor2*sum1;
    			
    		}
    	}
    	//print("P1: "+Math.round(timeStart4-timeStart3)+" P2: "+Math.round(millis()-timeStart4));
    	//print(derivativeXPart1);
    	//print(derivativeXPart2);
    	//print(derivativeYPart1);
    	//print(derivativeYPart2);


        // derivative Part 3 - arc integral : Notes on OneNote Coverage18
        var derivativeXPart3 = 0; var derivativeYPart3 = 0;
        var deltaTheta = stepSize/this.senRange;

        var arcSections = this.getArcSections()[0];
        
        // temporary
        // // arcSections =  [[],[],[]];
        var constantFactor = this.senRange*Math.exp(-this.sensingDecayFactor*this.senRange)*deltaTheta;
        if(arcSections.length==0){
            arcSections = [[-1*Math.PI, Math.PI]]; // do a full scan
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                while(theta <= arcSections[j][1]){
                    var interestedPoint = new Point2(this.position.x + this.senRange*cos(theta), this.position.y + this.senRange*sin(theta)); 
                    if(isLineOfSight(this.position,interestedPoint)){
                        constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbability(interestedPoint,true));

                        derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                        derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           
                    }
                    theta = theta + deltaTheta;
                }
            }
        }else{
            
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                              
                while(theta <= arcSections[j][1]){
                    var interestedPoint = new Point2(this.position.x + this.senRange*cos(theta), this.position.y + this.senRange*sin(theta)); 
                    constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbability(interestedPoint,true));

                    derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                    derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           

                    theta = theta + deltaTheta;
                }
            }
        }

        derivativeXPart3 = constantFactor*derivativeXPart3;
        derivativeYPart3 = constantFactor*derivativeYPart3;

        
        ////print("d1= "+derivativeXPart1+"; d2= "+derivativeXPart2+"; d3= "+derivativeXPart3);
    	return ([derivativeXPart1+derivativeXPart2+derivativeXPart3,derivativeYPart1+derivativeYPart2+derivativeYPart3]);	
    }
    
    
    this.getDerivativesWithoutt = function(){//local yet exaustive maybe

        var stepSize = 10;
        if(descretizationLevel!=null && descretizationLevel!=NaN){
            stepSize = descretizationLevel;
        }else{
            stepSize = 10;
        }
        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        
        var derivativeXPart1 = 0; var derivativeXPart2 = 0;
        var derivativeYPart1 = 0; var derivativeYPart2 = 0;
        
        //var timeStart3 = millis();
        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){
                */
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        //print(p);
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if( eventDensity>0 && dist<this.senRange && dist!=0){
                    //ellipse(x,y,2,2);
                    var distX = this.position.x-interestedPoint.x;
                    var distY = this.position.y-interestedPoint.y;
                    
                    if(dist!=0){//otherwise not defined
                        var constantTerm = eventDensity*(1-this.detectionProbabilityWithoutt(interestedPoint,true))*(-1*this.sensingDecayFactor)*this.sensingModelFunctionWithoutt(interestedPoint)*areaFactor/dist;
                        if (distX!=0){
                            derivativeXPart1 = derivativeXPart1 + constantTerm*distX;
                        }
                        if (distY!=0){
                            derivativeYPart1 = derivativeYPart1 + constantTerm*distY;
                        }
                    }
                    
                }
            }
        }
        //print("P1: "+Math.round(millis()-timeStart3));
        //var timeStart4 = millis();
        

        //part2
        stepSize = stepSize/5;//for more resolution

        ////
        var s; var v_j; var n_j; var D_j; var pointArray;
        var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();
        if(nearbyNonReflexVertices.length==0){
            derivativeXPart2 = 0;
            derivativeYPart2 = 0;
        }
        else{//need to do the summation
            for(var j = 0; j < nearbyNonReflexVertices.length; j = j+2){
                s = this.position;
                v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
                
                var D_j = distP2(s,v_j);
                
                n_j = normalizeP2(new Point2(-(s.y-v_j.y),(s.x-v_j.x)));
                if(!isLineOfSight(s,plusP2(intermediateP2(s,v_j,-0.3),productP2(n_j,2)))){
                    n_j = productP2(n_j,-1);
                }//now take sign(nj.x)
                
                //sin(theta_j)/D_j; theta_j between [0,pi/2];
                var factor1 = Math.sign(n_j.x)*abs(v_j.y-s.y)/sq(D_j);
                var factor2 = Math.sign(n_j.y)*abs(v_j.x-s.x)/sq(D_j);
                
                
                
                pointArray = pointsBeyondP2(this,v_j,stepSize);
                pointArray.pop();

                //fill(120);
                //print(pointArray);
                //printPointArrayP2(pointArray,"red",2);
                
                var sum1 = 0;
                for(var i =0; i<pointArray.length; i++){
                    var x = pointArray[i];//phro_j(r)
                    var r = distP2(x,v_j);
                    
                    sum1 = sum1 + getEventDensity(x)*(1-this.detectionProbabilityWithoutt(x,true))*this.sensingModelFunctionWithoutt(x)*r*stepSize; 
                }
                
                derivativeXPart2 = derivativeXPart2 + factor1*sum1;
                derivativeYPart2 = derivativeYPart2 + factor2*sum1;
                
            }
        }
        //print("P1: "+Math.round(timeStart4-timeStart3)+" P2: "+Math.round(millis()-timeStart4));
        //print(derivativeXPart1);
        //print(derivativeXPart2);
        //print(derivativeYPart1);
        //print(derivativeYPart2);


        // derivative Part 3 - arc integral : Notes on OneNote Coverage18
        var derivativeXPart3 = 0; var derivativeYPart3 = 0;
        var deltaTheta = stepSize/this.senRange;

        var arcSections = this.getArcSections()[0];
        
        // temporary
        // // arcSections =  [[],[],[]];
        var constantFactor = this.senRange*Math.exp(-this.sensingDecayFactor*this.senRange)*deltaTheta;
        if(arcSections.length==0){
            arcSections = [[-1*Math.PI, Math.PI]]; // do a full scan
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                while(theta <= arcSections[j][1]){
                    var interestedPoint = new Point2(this.position.x + this.senRange*cos(theta), this.position.y + this.senRange*sin(theta)); 
                    if(isLineOfSight(this.position,interestedPoint)){
                        constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbabilityWithoutt(interestedPoint,true));

                        derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                        derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           
                    }
                    theta = theta + deltaTheta;
                }
            }
        }else{
            
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                              
                while(theta <= arcSections[j][1]){
                    var interestedPoint = new Point2(this.position.x + this.senRange*cos(theta), this.position.y + this.senRange*sin(theta)); 
                    constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbabilityWithoutt(interestedPoint,true));

                    derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                    derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           

                    theta = theta + deltaTheta;
                }
            }
        }

        derivativeXPart3 = constantFactor*derivativeXPart3;
        derivativeYPart3 = constantFactor*derivativeYPart3;

        
        ////print("d1= "+derivativeXPart1+"; d2= "+derivativeXPart2+"; d3= "+derivativeXPart3);
        return ([derivativeXPart1+derivativeXPart2+derivativeXPart3,derivativeYPart1+derivativeYPart2+derivativeYPart3]);   
    }


    this.getDerivativesWithBoosting = function(){

        var stepSize = 10;
        if(descretizationLevel!=null && descretizationLevel!=NaN){
            stepSize = descretizationLevel;
        }else{
            stepSize = 10;
        }

        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        
        var derivativeXPart1 = 0; var derivativeXPart2 = 0;
        var derivativeYPart1 = 0; var derivativeYPart2 = 0;

        var derivativet = 0;
        
        //initialize variables with dummy values
        var interestedPoint = new Point2(0,0);  var eventDensity=0; var dist=0;
        var distX=0; var distY=0; var constantTermW1=0;

        //boosting coefficients
        var alpha1 = 0; var beta1 = 0;
        //end boosting coefficients
        
        //var timeStart3 = millis();
        /*for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        //print(p);
        //print("dd0"+descretizationLevel+" "+derivativeXPart1+"  "+ derivativeYPart1);

        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                
                //fill(0);
                //ellipse(x,y,2,2);

                var interestedPoint = new Point2(x,y);
                var eventDensity = getEventDensity(interestedPoint);
                var dist = distP2(this.position,interestedPoint);
                
                if( eventDensity>0 && dist<this.senRange && dist!=0){
                    //ellipse(x,y,2,2);
                    distX = this.position.x-interestedPoint.x;
                    distY = this.position.y-interestedPoint.y;
                    
                    //if(dist==0 && distX==0||distY==0){print("ohhh it is needed"+dist==0);}
                    
                    constantTermW1 = eventDensity*(1-this.detectionProbability(interestedPoint,true))*(-1*this.sensingDecayFactor)*this.sensingModelFunction(interestedPoint)*areaFactor;
                    
                    //boosting mode update
                    if(this.isBoostingActivated==1||boostingActivated==1){
                        //print("boosted"+this.id);
                        alpha1 = this.boostingAlpha1(interestedPoint);
                        beta1 = this.boostingBeta1(interestedPoint);
                        
                    }
                    else{//boosting deactivated
                        alpha1 = 1;
                        beta1 = 0;
                    }

                    derivativeXPart1 = derivativeXPart1 + (constantTermW1*alpha1+beta1)*distX/dist;
                    derivativeYPart1 = derivativeYPart1 + (constantTermW1*alpha1+beta1)*distY/dist;
                    
                    derivativet = derivativet + constantTermW1/((-1)*sensingDecayFactor*this.t);

                }
            }
        }
        
        //print("dd "+descretizationLevel+" "+derivativeXPart1+"  "+ derivativeYPart1);

        //boosting method 2 (neighbour offset adding)
        if((this.isBoostingActivated==1||boostingActivated==1) && boostingMethod==2){
            
            var lineOfSightNeighbours = this.lineOfSightNeighbours();//method 1 to select K
            var offsetValues = this.neighbourBoostingOffset(lineOfSightNeighbours);  //method 1
            
            /*var closestNeighbourIndex = this.closestNeighbour();//method 2 to select K          
            var offsetValues = this.neighbourBoostingOffset(closestNeighbourIndex);  //method 2*/

            derivativeXPart1 = derivativeXPart1 + offsetValues[0];
            derivativeYPart1 = derivativeYPart1 + offsetValues[1];
        }
        //boosting method 2 (neighbour offset adding)





        //print("P1: "+Math.round(millis()-timeStart3));
        //var timeStart4 = millis();
        

        // derivative part2

        //boosting coefficients
        var alpha2 = 0; var beta2 = 0; 
        stepSize = stepSize/5;//for more resolution

        ////
        var s; var v_j; var n_j; var D_j; var pointArray;
        var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();
        if(nearbyNonReflexVertices.length==0){
            derivativeXPart2 = 0;
            derivativeYPart2 = 0;
        }
        else{//need to do the summation
            

            // update this.nearbyNonReflexVerticesFound
            // check whether this vertex is already on the set
            // var activeJ = this.updateNearbyNonReflexVerticesFound(nearbyNonReflexVertices);
            
            for(var j = 0; j < nearbyNonReflexVertices.length; j = j+2){
                s = this.position;
                v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
                
                D_j = distP2(s,v_j);
                
                n_j = normalizeP2(new Point2(-(s.y-v_j.y),(s.x-v_j.x)));
                if(!isLineOfSight(s,plusP2(intermediateP2(s,v_j,-0.3),productP2(n_j,2)))){
                    n_j = productP2(n_j,-1);
                }//now take sign(nj.x)
                
                
                //sin(theta_j)/D_j; theta_j between [0,pi/2];
                var factor1 = Math.sign(n_j.x)*abs(v_j.y-s.y)/sq(D_j);
                var factor2 = Math.sign(n_j.y)*abs(v_j.x-s.x)/sq(D_j);
                
                
                
                pointArray = pointsBeyondP2(this,v_j,stepSize);
                pointArray.pop();

                //fill(120);
                //print(pointArray);
                ////printPointArrayP2(pointArray,"red",2);
                
                var sum1 = 0; var x; var r; var constantTermW2;

                for(var i =0; i<pointArray.length; i++){
                    x = pointArray[i];//phro_j(r)
                    r = distP2(x,v_j);
                    
                    //boosting coefficients loading
                    if(this.isBoostingActivated==1||boostingActivated==1){
                        alpha2 = 1;//this.boostingAlpha1(x);//this.boostingAlpha1(x);
                        beta2 = 0;//this.boostingAlpha1(x);//this.boostingAlpha1(x);//this.boostingBeta2(x);
                    }
                    else{//boosting deactivated
                        alpha2 = 1;
                        beta2 = 0;
                    }

                    // this is correct
                    ////constantTermW2 = getEventDensity(x)*(1-this.detectionProbability(x,true))*this.sensingModelFunctionForARay(x);
                    // this is wrong - try to avoid the following line and try to use the line above
                    constantTermW2 = getEventDensity(x)*(1-this.detectionProbability(x,true))*this.sensingModelFunction(x);


                    sum1 = sum1 + (alpha2*constantTermW2+beta2)*r*stepSize; 
                    //print(this.sensingModelFunctionForARay(x))
                }
                
                derivativeXPart2 = derivativeXPart2 + factor1*sum1;
                derivativeYPart2 = derivativeYPart2 + factor2*sum1;

                //boosting method 5 (V Boosting offset adding)
                if((this.isBoostingActivated==1||boostingActivated==1) && boostingMethod==5){
                    
                    //print(this.nearbyNonReflexVerticesFound);

                    // get gamma2, k2 form user interface, for now use
                    var gamma2 = boostingParameterY[1];//1;//2; 
                    //var gamma2 = 1;//1;//2; 
                    ///var gamma2 = 1;//2; 
                    var k2 = boostingParameterK[1];//5;//10;
                    //var k2 = 5;//5;//10;
                    //print(gamma2,k2)
                    ///var k2 = 2;//10;
                    var Vj = pointArray[pointArray.length-1];
                    
                    
                    //print(factor1*k2/(2*ej.x*Math.pow(D_j,gamma2)),factor2*k2/(2*ej.y*Math.pow(D_j,gamma2)));
                    //print(nearbyNonReflexVertices.length)
                    //print(activeJ,j)
                    if(typeof(Vj)=='undefined'){

                    ////
                    }else if(nearbyNonReflexVertices.length==2){// only when one non reflex vertex is there
                    //}else if(activeJ==j){
                        //var ej = normalizeP2(minusP2(Vj,v_j));
                        print("here",j);
                        derivativeXPart2 = derivativeXPart2 + factor1*k2*distP2(v_j,Vj)*Math.pow(distP2(s,Vj),gamma2);
                        derivativeYPart2 = derivativeYPart2 + factor2*k2*distP2(v_j,Vj)*Math.pow(distP2(s,Vj),gamma2);
                        
                        ///factor is correct (attractive force), but force is not balanced
                        ///derivativeXPart2 = derivativeXPart2 +(-1)*Math.sign(n_j.x)*abs(v_j.x-s.x)/(D_j)*k2*sum1;
                        ///derivativeYPart2 = derivativeYPart2 +Math.sign(n_j.y)*abs(v_j.y-s.y)/(D_j)*k2*sum1;
                    }
                    else{
                        //derivativeXPart2 = 0;
                        //derivativeYPart2 = 0;
                    }
                    
                    
                }
                
            }
        }
        //print("P1: "+Math.round(timeStart4-timeStart3)+" P2: "+Math.round(millis()-timeStart4));


        // derivative Part 3 - arc integral : Notes on OneNote Coverage18
        var derivativeXPart3 = 0; var derivativeYPart3 = 0;
        var deltaTheta = stepSize/this.senRange;

        var arcSections = this.getArcSections()[0];
        var alpha3 = 1; var beta3 = 0;

        //var constantFactor = senRange*Math.exp(-sensingDecayFactor*senRange)*deltaTheta; 
        var constantFactor = this.t*this.senRange*Math.exp(-this.sensingDecayFactor*this.senRange)*deltaTheta; 

        if(arcSections.length==0){
            arcSections = [[-1*Math.PI, Math.PI]]; // do a full scan and increment whatever here itself
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                while(theta <= arcSections[j][1]){
                    var interestedPoint = new Point2(this.position.x + this.senRange*cos(theta), this.position.y + this.senRange*sin(theta)); 
                    if(isLineOfSight(this.position,interestedPoint)){
                        constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbability(interestedPoint,true));

                        derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                        derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           
                    }
                    theta = theta + deltaTheta;
                }
            }
            
            derivativeXPart3 = constantFactor*derivativeXPart3;
            derivativeYPart3 = constantFactor*derivativeYPart3;
            
        }else{
            
            // select non covered arc section
            var minArcIndex; var maxArcIndex; var chosenPositiveArcs = []; var chosenNegativeArcs = [];
            if((this.isBoostingActivated==1||boostingActivated==1) && boostingMethod==6){
                
                var arcWidths = [];
                var arcAverageCoverageLevels = [];
                
                // when arcSections[j][0]==-Pi we need to append the last average!
                var jointArcExists = false;
                if(arcSections[0][0]==-PI && arcSections[arcSections.length-1][1]==PI){
                    //print("agent "+this.id+" has a joint arc");
                    jointArcExists = true;
                }

                for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                    var theta = arcSections[j][0] + (deltaTheta/2);    
                    var arcAverageCoverageLevel = 0;
                    var count1 = 0;
                    
                    while(theta <= arcSections[j][1]){
                        var interestedPoint = new Point2(this.position.x + (this.senRange-2)*cos(theta), this.position.y + (this.senRange-2)*sin(theta)); 
                        arcAverageCoverageLevel = arcAverageCoverageLevel + this.detectionProbability(interestedPoint,false);//this.boostingAlpha3(interestedPoint);
                        count1 = count1 + 1;
                        theta = theta + deltaTheta;
                    }

                    arcWidths.push(count1);
                    
                    if(count1>0){
                        arcAverageCoverageLevels.push(round(1000*arcAverageCoverageLevel/count1)/1000);
                    }else{
                        arcAverageCoverageLevels.push(0);
                    }

                }

                if(jointArcExists){// correction for the joint averages
                    var numOfArcSections = arcSections.length;
                    var jointArcAverage = (arcWidths[0]*arcAverageCoverageLevels[0] + arcWidths[numOfArcSections-1]*arcAverageCoverageLevels[numOfArcSections-1])/(arcWidths[0]+arcWidths[numOfArcSections-1]);
                    arcAverageCoverageLevels[0] = jointArcAverage;
                    arcAverageCoverageLevels[numOfArcSections-1] = jointArcAverage;
                }
                //need to select the best arc and the worst arc
                //print("Arc averages = "+arcAverageCoverageLevels);
                //print("Arc widths = "+arcWidths);
                minArcIndex = arcAverageCoverageLevels.indexOf(min(arcAverageCoverageLevels));
                maxArcIndex = arcAverageCoverageLevels.indexOf(max(arcAverageCoverageLevels));
                if(Math.abs(arcAverageCoverageLevels[maxArcIndex]-arcAverageCoverageLevels[minArcIndex])>0.025){

                    for(var j=0; j<arcAverageCoverageLevels.length; j++){
                        if(Math.abs(arcAverageCoverageLevels[j]-arcAverageCoverageLevels[minArcIndex])<0.001){
                            chosenPositiveArcs.push(j);
                        }else if(Math.abs(arcAverageCoverageLevels[j]-arcAverageCoverageLevels[maxArcIndex])<0.001){
                            chosenNegativeArcs.push(j);   
                        }
                    }

                }

                // assisting neighborless agents
                if(chosenNegativeArcs.length==0 && chosenPositiveArcs.length==0 && arcSections.length>0){
                    var effectiveNeighbors = this.getEffectiveNeighbors();
                    //print(effectiveNeighbors);
                    //print("here");

                    if(effectiveNeighbors.length<2){// history comes into play
                        //print("here");
                        var pastLastTrackPoint = this.pastTrackPoints[this.pastTrackPoints.length-1];
                        if(this.pastTrackPoints.length>1){
                            pastLastTrackPointTemp = this.pastTrackPoints[this.pastTrackPoints.length-2];
                            pastLastTrackPoint = productP2(plusP2(pastLastTrackPoint,pastLastTrackPointTemp),0.5);
                        }
                        

                        var theta = Math.atan2(pastLastTrackPoint.y-this.position.y,pastLastTrackPoint.x-this.position.x);
                        //print("theta "+theta);
                        for(var j = 0; j<arcSections.length; j++){//considering each arc section
                            var deviation = Math.abs(0.5*(arcSections[j][0]+arcSections[j][1])-theta);
                            if((arcSections[j][0]<theta && theta<arcSections[j][1])||(deviation<=(Math.PI/4))){
                                chosenNegativeArcs.push(j);
                            }else{
                                chosenPositiveArcs.push(j);
                            }
                        }
                    }// end - history comes into play
                }
                
                //print("Difference = "+(arcAverageCoverageLevels[maxArcIndex]-arcAverageCoverageLevels[minArcIndex]));

            }
            // end of select non covered arc section

            
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                var pointArrayP = [];
                var pointArrayN = [];
                var pointArrayNull =[];

                while(theta <= arcSections[j][1]){
                    var interestedPoint = new Point2(this.position.x + (this.senRange-1)*cos(theta), this.position.y + (this.senRange-1)*sin(theta)); 
                    var constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbability(interestedPoint,true));
                    //print("con "+constantTerm);
                    //print("boostK"+boostingParameterK);
                    if((this.isBoostingActivated==1||boostingActivated==1) && boostingMethod==6){
                        if(chosenPositiveArcs.includes(j)){//boosting activated
                            alpha3 = 0;//-this.boostingAlpha3(interestedPoint);
                            beta3 = 1*boostingParameterK*this.boostingBeta3(interestedPoint)*deltaTheta/constantFactor;
                            pointArrayP.push(interestedPoint);                            
                        }else if(chosenNegativeArcs.includes(j)){
                            alpha3 = 0;//this.boostingAlpha3(interestedPoint);
                            beta3 = -1*boostingParameterK*this.boostingBeta3(interestedPoint)*deltaTheta/constantFactor;
                            pointArrayN.push(interestedPoint);    
                        }else{
                            alpha3 = 1;
                            beta3 = 0;  
                            pointArrayNull.push(interestedPoint);  
                        }

                    }
                    else{//boosting deactivated
                        alpha3 = 1;
                        beta3 = 0;
                        //pointArrayNull.push(interestedPoint); 
                    }

                    derivativeXPart3 = derivativeXPart3 + (constantTerm*alpha3+beta3)*cos(theta);           
                    derivativeYPart3 = derivativeYPart3 + (constantTerm*alpha3+beta3)*sin(theta);           

                    theta = theta + deltaTheta;     
                }
                noStroke();
                fill(100);
                printPointArrayP2(pointArrayP,"red",2);
                printPointArrayP2(pointArrayN,"blue",2);
                printPointArrayP2(pointArrayNull,"green",2);
            }

            derivativeXPart3 = constantFactor*derivativeXPart3;
            derivativeYPart3 = constantFactor*derivativeYPart3;

        }
        // END derivative part 3 

        if(boostingActivated==1 && boostingMethod==4){//random purtubation technique
            //print(round(derivativeXPart1+derivativeXPart2),round(derivativeYPart1+derivativeYPart2));
            //boostingActivated = false;
            ////return ([derivativeXPart1+derivativeXPart2+(2*boostingParameterK*Math.random()-boostingParameterK),derivativeYPart1+derivativeYPart2+(2*boostingParameterK*Math.random()-boostingParameterK)]); 
            return ([derivativeXPart1+derivativeXPart2+(2*boostingParameterK*Math.random()-boostingParameterK),derivativeYPart1+derivativeYPart2+(2*boostingParameterK*Math.random()-boostingParameterK),derivativet+weightPGD*this.agentCostRatio*this.sensingCapacity]); 
        }else{
            /*print("dx"+this.id);
            print(round(derivativeXPart1),round(derivativeXPart2),round(derivativeXPart3));
            print("dy"+this.id);
            print(round(derivativeYPart1),round(derivativeYPart2),round(derivativeYPart3));*/
            ////return ([derivativeXPart1+derivativeXPart2+derivativeXPart3,derivativeYPart1+derivativeYPart2+derivativeYPart3]);
            return ([derivativeXPart1+derivativeXPart2+derivativeXPart3,derivativeYPart1+derivativeYPart2+derivativeYPart3,derivativet+weightPGD*this.agentCostRatio*this.sensingCapacity]);

        }
    }



    this.updateNearbyNonReflexVerticesFound = function(nearbyNonReflexVertices){ // this is meant for an update of V - boosting
        var activeJ=-1;//keeps the index os active vertex out of nearbyNonRedlexVertices
        
        if(this.nearbyNonReflexVerticesFound.length==0){
            
            if((this.isBoostingActivated==1||boostingActivated==1) && boostingMethod==5){ // Vboosting is activated for the first time
                var distances = [];
                for (var j = 0; j<nearbyNonReflexVertices.length; j=j+2) {
                    var v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
                    this.nearbyNonReflexVerticesFound.push([-1,nearbyNonReflexVertices[j],nearbyNonReflexVertices[j+1],v_j]);
                    distances.push(distP2(this.position, v_j));
                }
                // activating one NRV
                var maxIndex = distances.indexOf(max(distances));
                this.nearbyNonReflexVerticesFound[maxIndex][0] = 1;
                activeJ = maxIndex;
                return activeJ;

            }
        }
        else if((this.isBoostingActivated==1||boostingActivated==1) && boostingMethod==5){//boosting activated, have few nonreflex vertices already 
            // if it is different from earlier ones, use ut
            var distances =[];
            var activeJOld=-1;
            var activeNewJs = [];
            var activeNewIs = [];
            var distanceOld;
            var activeIOld;

            var verticesFoundSoFar = this.nearbyNonReflexVerticesFound;
            for (var j = 0; j<nearbyNonReflexVertices.length; j=j+2) {

                var v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];//can be new or old
                
                var equalFound = -1;

                for(var i=0; i<verticesFoundSoFar.length;i++){
                    if(isEqualP2(verticesFoundSoFar[i][3],v_j)){//already in the past data
                        equalFound = i;
                        //print("equal");


                    }
                    if(verticesFoundSoFar[i][0]==1){//past active one
                        activeJOld = j;
                        activeIOld = i;
                        print("hey",i,j)
                        distanceOld = distP2(this.position, v_j);
                    }
                    

                }

                if(equalFound==-1){//v_j is a new one
                    this.nearbyNonReflexVerticesFound.push([-1,nearbyNonReflexVertices[j],nearbyNonReflexVertices[j+1],v_j]);
                    //print("new one",v_j);
                    activeNewJs.push(j);
                    activeNewIs.push(this.nearbyNonReflexVerticesFound.length-1);
                    distances.push(distP2(this.position, v_j));
                }
                
            }

            // activating one NRV
            var maxIndex = distances.indexOf(max(distances));
            if(max(distances)<distanceOld||distances.length==0){
                activeJ = activeJOld;
                print("now")
            }
            else{
                activeJ = activeNewJs[maxIndex];
                this.nearbyNonReflexVerticesFound[activeNewIs[maxIndex]][0] = 1;
                for(var i=0; i<this.nearbyNonReflexVerticesFound.length;i++){
                    if(i!=activeNewIs[maxIndex]){
                        this.nearbyNonReflexVerticesFound[i][0] = -1;
                    }
                }
            }
            
            //activeJ = maxIndex;
            return activeJ;

            //stick to active one till the stability
        }else{//boosting is not activated, but past vertexes have been used

            print("this");
        }
        //print("this1");
        return activeJ;
    }



    // boosting functions
    this.boostingAlpha1 = function(interestedPoint){
        //boostingMethod;
        if(boostingMethod==1){//P - boosting
            //var K = 1;
            //var gamma = 1;
            var P = this.detectionProbability(interestedPoint,false);
            //print(P);
            //print(K*Math.pow(this.detectionProbability(interestedPoint,false),-1*gamma));
            if(P > 0.01){
                return boostingParameterK*Math.pow(P,-1*boostingParameterY);
            }else{
                return 1;
            }
        }else if(boostingMethod==3){//Phi - boosting
            //var K = 10;
            //var gamma = 2;
            var Phi = 1-this.detectionProbability(interestedPoint,true);
            // print(Phi);
            //print(Phi);
            //print(K*Math.pow(this.detectionProbability(interestedPoint,false),-1*gamma));
            return boostingParameterK*Math.pow(Phi,boostingParameterY);
        }else if(boostingMethod==5){//V - boosting (Component of the Phi bosting)
            //var K = 10;
            //var gamma = 2;
            var Phi = 1-this.detectionProbability(interestedPoint,false);
            // print(Phi);
            //print(Phi);
            //print(K*Math.pow(this.detectionProbability(interestedPoint,false),-1*gamma));
            return boostingParameterK[0]*Math.pow(Phi,boostingParameterY[0]);
        }else if(boostingMethod==7){
            if(this.targetVertexAnchor!==-1){// there is a target vertex anchor
                if(this.checkTargetVertexAnchorReached()){// agent achieved target!!
                    print('Reached!');
                    this.verticesFoundSoFarRegistry[this.targetVertexAnchorIndex][5] = true;// completed this
                    this.findTargetVertexAnchor();// find a new one !!!
                    return 1; // end reached
                }else if(distP2(interestedPoint,this.targetVertexAnchor)<50){//50 = radius of attraction region from Z_{ij\bar{k}}
                    var force = Math.pow(distP2(this.position,this.targetVertexAnchor),1)*Math.exp(-0.001*distP2(interestedPoint,this.targetVertexAnchor)); 
                    print('Agent '+this.id+' Applying : '+force);
                    return force;
                }else{
                    return 1; // point x out of range from target point Z_{ij\bar{k}}
                }
            }else{
                return 1; // no target => no force
            }
        }
        else{
            return 1;
        }
    }

    this.boostingBeta1 = function(interestedPoint,additionalArgument){
        //boostingMethod;
        return 0;
        
    }

    this.boostingAlpha2 = function(interestedPoint){
        //boostingMethod;
        return 1;
    }
    
    this.boostingAlpha3 = function(interestedPoint){
        //boostingMethod;
        if(boostingMethod==6){//arc boosting
            var P = this.detectionProbability(interestedPoint,false);// use "true" to get only neighbors
            //print("here "+ P);
            if(P > 0.0001){
                return boostingParameterK*Math.pow(P,-1*boostingParameterY);
            }else{
                return 1;
            }
        }else{
            return 1;
        }
    }
    
    this.boostingBeta2 = function(interestedPoint){
        //boostingMethod;
        return 0;
    }

    this.boostingBeta3 = function(interestedPoint){
        //boostingMethod;
        //return ((1-Math.exp(-1*sensingDecayFactor*senRange)*(1+senRange*sensingDecayFactor))/sensingDecayFactor);
        return ((1-Math.exp(-1*this.sensingDecayFactor*this.senRange/boostingParameterY)*(1+this.senRange*this.sensingDecayFactor/boostingParameterY))/(this.sensingDecayFactor/boostingParameterY));
        
    }
   
    this.checkTargetVertexAnchorReached = function(){// check whether agent wen close to the target Z_j
        //simplest method
        ////return distP2(this.position,this.targetVertexAnchor)<10
        var s = this.position;
        var z = this.targetVertexAnchor;
        var v = this.verticesFoundSoFarRegistry[this.targetVertexAnchorIndex][2];
        var s0 = this.verticesFoundSoFarRegistry[this.targetVertexAnchorIndex][3];
        if((s0.x <= v.x && v.x <= z.x && z.x <= s.x)||(s0.x >= v.x && v.x >= z.x && z.x >= s.x) || (s0.y <= v.y && v.y <= z.y && z.y <= s.y)||(s0.y >= v.y && v.y >= z.y && z.y >= s.y)){
            return true;
        }else if(distP2(s,z)<10){
            return true;
        }else{
            return false;
        }

    }
    
    this.closestNeighbour = function(){
        var neighbours = this.getNeighbors();
        var distances = [];
        for(var j = 0; j<neighbours.length; j++){
            distances[j] = round(10*distP2(this.position,particleShadows[neighbours[j]].position))/10;            
        }
        //print(distances);
        if(distances.length>0){
            return neighbours[distances.indexOf(min(distances))];
        }
        else{
            return -1;
        }
    }

    this.lineOfSightNeighbours = function(){
        var neighbours = this.getNeighbors();
        var lineOfSightNeighbours = [];
        var indexCount = 0;

        for(var j = 0; j<neighbours.length; j++){
            if(isLineOfSight(this.position,particleShadows[neighbours[j]].position)){
                lineOfSightNeighbours[indexCount] = neighbours[j];
                indexCount++;
            }       
        }
        //print(distances);
        return lineOfSightNeighbours;    
    }

    this.neighbourBoostingOffset = function(additionalArgument){
        
        //var K = 10000;
        //var gamma =  1;       

        // if method 1 uncomment following

        //method1: Kj = 5 if neighbour is visible; 0 otherwise; additionalArgument=lineOfSightNeighbours
        //I will implement this later
        //method 1 start
        if(typeof(additionalArgument)!="undefined"){
            var lineOfSightNeighbours = additionalArgument;
            var si = this.position;
            var tempSumX = 0;
            var tempSumY = 0;

            for(var j = 0; j<lineOfSightNeighbours.length; j ++ ){
                var sj = particleShadows[lineOfSightNeighbours[j]].position;
                var constantTermTemp = -1*boostingParameterK/pow(distP2(sj,si),(boostingParameterY+1));
                tempSumX = tempSumX + constantTermTemp*(sj.x-si.x);
                tempSumY = tempSumY + constantTermTemp*(sj.y-si.y);
            }
            //print(tempSumX,tempSumY);
            return [tempSumX,tempSumY];
        }else{
            //print("neighbour boosting offsets undefined method 1 for ID"+this.id);
            return [0,0];
        }
        //method 2 end



        // if method 2 uncomment following
        
        /*// method2: Kj = 5 only for the closest neighbour// most affective ; additionalArgument=closestNeighbourIndex
        // lets do it now
        // method 2 start
        if(typeof(additionalArgument)!="undefined"){
            var closestNeighbourIndex = additionalArgument;
            var sj = particleShadows[closestNeighbourIndex].position;
            var si = this.position;
            var constantTermTemp = -1*boostingParameterK/pow(distP2(sj,si),boostingParameterY+1);
            //print([constantTermTemp*(sj.x-si.x),constantTermTemp*(sj.y-si.y)]);
            return [constantTermTemp*(sj.x-si.x),constantTermTemp*(sj.y-si.y)];
        }else{
            //print("neighbour boosting offsets undefined method 2 for ID "+this.id);
            return [0,0];
        }
        // method 2 end*/
        
        //
        //return 0;

    }

    this.findTargetVertexAnchor = function(){
        // must return true if found a target, otherwise must return false
        // can use following variables
        // this.recentLocalCoverageLevels = [-1,0,0]; // can use to compare the coverage levels
        // this.verticesFoundSoFarRegistry = []; // [obstacleIndex,vertexIndex,v_j,Z_j,trackedOrNot]
        // trackedOrNot= vertices went around so far
        // this.targetVertexAnchor = -1;
        // this is executed only in the instance where boosting get activated

        var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();

        if(nearbyNonReflexVertices.length==0){// no vertices around !!!
            ////this.targetVertexAnchor = -1;
            ////this.targetVertexAnchorIndex = -1;
            print('No new vertices to check! Lets go through registry!');
            ////return false; // no targets
            // maybe we should use past infomation. - later -> later is now!!
            // lets scan the registry and if there is one that was not explored, lets go there!
            // will this work when multiple agents are already there
        }else{
            
            // need to update the registry
            if(this.verticesFoundSoFarRegistry.length==0){// registry is empty - load them up!
                
                var registry = [];
                for (var i = 0; i<nearbyNonReflexVertices.length; i=i+2){// take one by one new vertices
                    var entry = [];
                    var v_j = obstacles[nearbyNonReflexVertices[i]].nonReflexVertices[nearbyNonReflexVertices[i+1]];
                    entry.push(nearbyNonReflexVertices[i]); //obstacle index
                    entry.push(nearbyNonReflexVertices[i+1]); //obstacle's NR-vertex index
                    entry.push(v_j); //vertex coordinate
                    entry.push(this.position);// position where agent was
                    var pointArray = pointsBeyondP2(this,v_j,2);
                    pointArray.pop(); 
                    Z_j = pointArray[pointArray.length-1];
 
                    entry.push(Z_j); //Anchor point
                    entry.push(false); // has not being tracked before!
                    registry.push(entry);
                }
                this.verticesFoundSoFarRegistry = registry;
                print('New Registry: '+registry);
            
            }else{// check and load !!!
                
                var registry = this.verticesFoundSoFarRegistry;
                for (var i = 0; i<nearbyNonReflexVertices.length; i=i+2){// take one by one new vertices
                    var obstacleIndex = nearbyNonReflexVertices[i];
                    var vertexIndex = nearbyNonReflexVertices[i+1];    
                    var entry = [];
                    var existingEntryFound = false;
                    

                    for (var j=0; j<registry.length; j++){
                        if(registry[j][0] == obstacleIndex && registry[j][1] == vertexIndex){//existing record!
                            existingEntryFound = true;
                        }
                    }

                    if(!existingEntryFound){
                        var v_j = obstacles[obstacleIndex].nonReflexVertices[vertexIndex];
                        entry.push(obstacleIndex); //obstacle index
                        entry.push(vertexIndex); //obstacle's NR-vertex index
                        entry.push(v_j); //vertex coordinate
                        entry.push(this.position);// position where agent was
                        var pointArray = pointsBeyondP2(this.position,v_j,2);
                        pointArray.pop(); 
                        Z_j = pointArray[pointArray.length-1];
     
                        entry.push(Z_j); //Anchor point
                        entry.push(false); // has not being tracked before!
                        print('New entry added: '+entry);
                        if(pointArray.length>2){// z is not so close to v
                            this.verticesFoundSoFarRegistry.push(entry);
                        }
                    }
                    //v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
                }
            }
            // end - updating the registry !!!
        }

        // need to make a selection from the registry !


        // selecting new entries avoiding visited ones:
        var registry = this.verticesFoundSoFarRegistry;
        var neighborSet = this.getEffectiveNeighbors();
        for (var i = 0; i<registry.length; i++){
            if(!registry[i][5]){
                if(neighborSet.length == 0 && distP2(registry[i][4],this.position)>(this.senRange+10)){
                    print('Agent '+this.ID+' Relocated to follow a new VA');
                    this.position = registry[i][3];
                }
                this.targetVertexAnchor = registry[i][4];
                this.targetVertexAnchorIndex = i;


                print('Jump to next!');
                return true
            }  
        }
        this.targetVertexAnchor = -1;
        this.targetVertexAnchorIndex = -1;
        print('All covered !');
        return false;

        // end - making selection





    }

    this.getSecondDerivativeWRT = function(neighborAgent){

        var stepSize = descretizationLevel;

        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        
        //initialize variables with dummy values
        var interestedPoint = new Point2(0,0);  var eventDensity=0; 
        var disti=0; var distj = 0;
        var distiX=0; var distiY=0; 
        var distjX=0; var distjY=0; 

        var p_i = 0;
        var p_j = 0;

        var constantTerm=0;
        var result = new Point2(0,0); //sum(row 1):x ; sum(row 2)  = y

        if(neighborAgent==this.id){// with respect to itself: Terms of type 1,2,and 3
            
            var crossDerivative = 0;    // d^2H_i(s) / ds_ix.ds_iy
            var secondDerivativeX = 0;  // d^2H_i(s) / ds_ix^2
            var secondDerivativeY = 0;  // d^2H_i(s) / ds_iy^2

            //** area integral
            var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
            for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
                for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                    //fill(0);
                    //ellipse(x,y,2,2);
                    interestedPoint = new Point2(x,y);
                    eventDensity = getEventDensity(interestedPoint);
                    disti = distP2(this.position,interestedPoint);
                    
                    if(eventDensity > 0 && disti < this.senRange && disti!=0){
                        
                        p_i = this.sensingModelFunction(interestedPoint);

                        if(p_i > 0){
                            //ellipse(x,y,2,2);
                            distiX = this.position.x-interestedPoint.x;
                            distiY = this.position.y-interestedPoint.y;
                            
                            //constantTermW1 = eventDensity*(1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent))*(this.sensingDecayFactor)*p_i*p_j*areaFactor;
                            constantTerm = eventDensity*(1-this.detectionProbability(interestedPoint,true))*(this.sensingDecayFactor)*p_i*areaFactor/sq(disti);
                            //print(constantTerm);
                            crossDerivative = crossDerivative + constantTerm*(distiX*distiY*(this.sensingDecayFactor + (1/disti)));
                            secondDerivativeX = secondDerivativeX + constantTerm*(this.sensingDecayFactor*sq(distiX)-(sq(distiY)/disti));
                            secondDerivativeY = secondDerivativeY + constantTerm*(this.sensingDecayFactor*sq(distiY)-(sq(distiX)/disti));
                        }                        
                    }
                }
            }
            //** end area integral




            //** line integrals - non-reflex vertices - (Coverage 18)
            stepSize = stepSize/5;//for more resolution

            var s; var v_j; var n_j; var D_j; var pointArray;
            var secondDerivativeXPart2=0; var secondDerivativeYPart2 = 0; var crossDerivativePart2 = 0;
            
            var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();
            if(nearbyNonReflexVertices.length==0){
                // no need to add anything to cross or seccong derivatives
            }
            else{//need to do the summation
                
                for(var j = 0; j < nearbyNonReflexVertices.length; j = j+2){
                    s = this.position;
                    v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
                    
                    D_j = distP2(s,v_j);
                    
                    n_j = normalizeP2(new Point2(-(s.y-v_j.y),(s.x-v_j.x)));
                    if(!isLineOfSight(s,plusP2(intermediateP2(s,v_j,-0.3),productP2(n_j,2)))){
                        n_j = productP2(n_j,-1);
                    }//now take sign(nj.x)
                    
                    
                    //sin(theta_j)/D_j; theta_j between [0,pi/2];
                    var factor1 = Math.sign(n_j.x)*abs(v_j.y-s.y)/sq(D_j);
                    var factor2 = Math.sign(n_j.y)*abs(v_j.x-s.x)/sq(D_j);
                    
                    
                    
                    pointArray = pointsBeyondP2(this,v_j,stepSize);
                    pointArray.pop();
                    
                    var sumX = 0; var sumY=0; var x; var r; var constantTerm=0;

                    for(var i =0; i<pointArray.length; i++){
                        x = pointArray[i];//phro_j(r)
                        r = distP2(x,v_j);
                        
                        if(distP2(s,x)!=0){
                            constantTerm = -1*this.sensingDecayFactor*getEventDensity(x)*(1-this.detectionProbability(x,true))*this.sensingModelFunction(x)*r*stepSize/distP2(s,x);
                        }else{
                            constantTerm = 0;
                        }

                        sumX = sumX + constantTerm*(s.x-x.x); 
                        sumY = sumY + constantTerm*(s.y-x.y); 
                        //print(this.sensingModelFunctionForARay(x))
                    }
                    
                    secondDerivativeXPart2 = secondDerivativeXPart2 + factor1*sumX;
                    secondDerivativeYPart2 = secondDerivativeYPart2 + factor2*sumY;
                    crossDerivativePart2 = crossDerivativePart2 + factor2*sumX;
                }
            }
            //** end line integrals - non-reflex vertices - 


            //** circular integrals - from sensing range
            var secondDerivativeXPart3 = 0; var secondDerivativeYPart3 = 0; var crossDerivativePart3 = 0;
            var deltaTheta = stepSize/this.senRange;

            var arcSections = this.getArcSections()[0];
            
            if(arcSections.length==0){
                // no need to do anything
            }else{ // have arcs!!
                
                var constantFactor = -1*this.sensingDecayFactor*Math.exp(-this.sensingDecayFactor*this.senRange)*deltaTheta/this.senRange; 
                var s = this.position;

                for(var j = 0; j<arcSections.length; j++){// considering each arc section
                    
                    var theta = arcSections[j][0] + (deltaTheta/2);
                    var constantTerm = 0;          
                    
                    while(theta <= arcSections[j][1]){
                        var x = new Point2(s.x + this.senRange*cos(theta), s.y + this.senRange*sin(theta)); 
                        constantTerm = getEventDensity(x)*(1-this.detectionProbability(x,true));

                        secondDerivativeXPart3 = secondDerivativeXPart3 + constantTerm*(s.x-x.x)*cos(theta);           
                        secondDerivativeYPart3 = secondDerivativeYPart3 + constantTerm*(s.y-x.y)*sin(theta);
                        crossDerivativePart3 = crossDerivativePart3 + constantTerm*(s.x-x.x)*sin(theta);           

                        theta = theta + deltaTheta;
                    }
                }

                secondDerivativeXPart3 = secondDerivativeXPart3*constantFactor;
                secondDerivativeYPart3 = secondDerivativeYPart3*constantFactor;
                crossDerivativePart3 = crossDerivativePart3*constantFactor;
                
            }
            //** end circular integrals

            secondDerivativeX = secondDerivativeX + secondDerivativeXPart2 + secondDerivativeXPart3; 
            secondDerivativeY = secondDerivativeY + secondDerivativeYPart2 + secondDerivativeYPart3; 
            secondDerivative = crossDerivative + crossDerivativePart2 + crossDerivativePart3; 


            result.x = Math.abs(secondDerivativeX) + Math.abs(crossDerivative);
            result.y = Math.abs(secondDerivativeY) + Math.abs(crossDerivative);
            return result;

        }else{// terms in type 4,5,6, and 7
            
            var crossDerivative_jxix = 0;    // d^2H_i(s) / ds_jx.ds_ix
            var crossDerivative_jyix = 0;    // d^2H_i(s) / ds_jy.ds_ix
            var crossDerivative_jxiy = 0;    // d^2H_i(s) / ds_jx.ds_iy
            var crossDerivative_jyiy = 0;    // d^2H_i(s) / ds_jy.ds_iy
            
            var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
            for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
                for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                    //fill(0);
                    //ellipse(x,y,2,2);
                    interestedPoint = new Point2(x,y);
                    eventDensity = getEventDensity(interestedPoint);
                    disti = distP2(this.position,interestedPoint);
                    distj = distP2(particleShadows[neighborAgent].position,interestedPoint);

                    if(eventDensity > 0 && disti < this.senRange && distj < this.senRange && disti!=0 && distj!=0){
                            
                        p_i = this.sensingModelFunction(interestedPoint);
                        p_j = particleShadows[neighborAgent].sensingModelFunction(interestedPoint);
                        
                        if(p_i>0 && p_j>0){
                            //ellipse(x,y,2,2);
                            distiX = this.position.x-interestedPoint.x;
                            distiY = this.position.y-interestedPoint.y;

                            distjX = particleShadows[neighborAgent].position.x - interestedPoint.x;
                            distjY = particleShadows[neighborAgent].position.y - interestedPoint.y;
                            
                            constantTerm = eventDensity*(1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent))*(-1*sq(this.sensingDecayFactor))*p_i*p_j*areaFactor/(disti*distj);
                             
                            crossDerivative_jxix = crossDerivative_jxix + constantTerm*distiX*distjX;    // d^2H_i(s) / ds_jx.ds_ix
                            crossDerivative_jyix = crossDerivative_jyix + constantTerm*distiX*distjY;    // d^2H_i(s) / ds_jy.ds_ix
                            crossDerivative_jxiy = crossDerivative_jxiy + constantTerm*distiY*distjX;    // d^2H_i(s) / ds_jx.ds_iy
                            crossDerivative_jyiy = crossDerivative_jyiy + constantTerm*distiY*distjY;    // d^2H_i(s) / ds_jy.ds_iy
                        
                        }
                        
                    }
                }
            }

            result.x = Math.abs(crossDerivative_jxix) + Math.abs(crossDerivative_jyix);
            result.y = Math.abs(crossDerivative_jxiy) + Math.abs(crossDerivative_jyiy);            
            return result;
        }   
    }

    this.tuneLipschitzConstant = function(){

        var neighbors = this.getEffectiveNeighbors();
        var candidates = this.getSecondDerivativeWRT(this.id);
        
        //print(candidates);
        for(var i = 0; i<neighbors.length; i++){
            candidates = plusP2(candidates,this.getSecondDerivativeWRT(neighbors[i]));
        }
        var lipschitzConstantK1Temp = ceil(1000000*max([candidates.x , candidates.y]))/1000000;
        if(lipschitzConstantK1Temp>0){
            this.lipschitzConstantK1 = lipschitzConstantK1Temp;
            //print("Lipschitz "+this.lipschitzConstantK1);
        }else{
            print("Error evaluating K_1:"+candidates.x);
            this.lipschitzConstantK1 = lipschitzConstantK1;
        }
        consolePrint("Agent "+(this.id+1)+"'s Lipschitz constant updated as: "+round(1000*this.lipschitzConstantK1)/1000);
    }

    this.getJointDerivative = function(neighborAgent,modeOfNeighborAgent){//neighborAgent = j
        
        var stepSize = 10;
        if(obstacles.length>0 && descretizationLevel!=null && descretizationLevel!=NaN){
            stepSize = descretizationLevel;
        }else{
            stepSize = descretizationLevel;
        }

        var areaFactor = sq(stepSize);
        var halfStepSize = stepSize/2;
        
        var derivativeXPart1 = 0; var derivativeYPart1 = 0;
        var derivativeXPart2 = 0; var derivativeYPart2 = 0;
        var derivativeXPart3 = 0; var derivativeYPart3 = 0;
        
        var derivativet = 0;

        //initialize variables with dummy values
        var interestedPoint = new Point2(0,0);  var eventDensity=0; var dist=0;
        var distX=0; var distY=0; var constantTermW1=0;

        //boosting coefficients
        var alpha1 = 1; var beta1 = 0;
        //end boosting coefficients
        
        var p_i = 0;
        var p_j = 0;

        //var timeStart3 = millis();
        /*
        for (var x = this.position.x-senRange+halfStepSize; x<=this.position.x+senRange-halfStepSize; x+=stepSize){
            for(var y = this.position.y-senRange+halfStepSize; y<=this.position.y+senRange-halfStepSize; y+=stepSize){*/
        var p = floor(sqrt(sq(this.senRange/halfStepSize)-1));
        for (var x = this.position.x-halfStepSize*p; x<=this.position.x+halfStepSize*p; x+=stepSize){
            for(var y = this.position.y-halfStepSize*p; y<=this.position.y+halfStepSize*p; y+=stepSize){
                //fill(0);
                //ellipse(x,y,2,2);
                interestedPoint = new Point2(x,y);
                eventDensity = getEventDensity(interestedPoint);
                dist = distP2(this.position,interestedPoint);
                
                if(eventDensity > 0 && dist < this.senRange && dist!=0){
                    
                    p_i = this.sensingModelFunction(interestedPoint);
                    p_j = particleShadows[neighborAgent].sensingModelFunction(interestedPoint);
                    
                    if(p_i>0 && p_j>0){
                        //ellipse(x,y,2,2);
                        distX = this.position.x-interestedPoint.x;
                        distY = this.position.y-interestedPoint.y;
                        //if(dist==0||distX==0||distY==0){print("ohhh it is needed"+dist==0);}
                        if(modeOfNeighborAgent==0){
                            constantTermW1 = eventDensity*(1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent))*(this.sensingDecayFactor)*p_i*p_j*areaFactor;
                        }else if(decentralizedBoostingMethod!=1 || boostingMethod!=3){//not Phi boosting nor decentralized.
                            constantTermW1 = eventDensity*(1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent))*(this.sensingDecayFactor)*p_i*p_j*areaFactor;
                        }else{//neighbor is in boosting mode
                            //print("Phi boosting!!!");
                            constantTermW1 = boostingParameterK*(1+boostingParameterY)*(this.sensingDecayFactor)*eventDensity*p_j*pow((1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent)),boostingParameterY+1)*pow(1-p_i,boostingParameterY)*p_i*areaFactor;
                        }

                        derivativeXPart1 = derivativeXPart1 + (constantTermW1)*distX/dist;
                        derivativeYPart1 = derivativeYPart1 + (constantTermW1)*distY/dist;

                        derivativet = derivativet + constantTermW1/((-1)*this.sensingDecayFactor*this.t);
                    
                    }
                    
                }
            }
        }

        // new parts 

        // derivative part2
        stepSize = stepSize/5;//for more resolution

        //// nonreflex vertices of agent  i (not neighbor j's)
        var s; var v_j; var n_j; var D_j; var pointArray;
        var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();

        if(nearbyNonReflexVertices.length==0){
            derivativeXPart2 = 0;
            derivativeYPart2 = 0;
        }
        else{//need to do the summation

            for(var j = 0; j < nearbyNonReflexVertices.length; j = j+2){
                
                s = this.position;
                v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
                
                D_j = distP2(s,v_j);
                
                n_j = normalizeP2(new Point2(-(s.y-v_j.y),(s.x-v_j.x)));
                if(!isLineOfSight(s,plusP2(intermediateP2(s,v_j,-0.3),productP2(n_j,2)))){
                    n_j = productP2(n_j,-1);
                }//now take sign(nj.x)
                                
                //sign(nj.x)*sin(theta_j)/D_j; theta_j between [0,pi/2];
                var factor1 = -1*Math.sign(n_j.x)*abs(v_j.y-s.y)/sq(D_j);
                var factor2 = -1*Math.sign(n_j.y)*abs(v_j.x-s.x)/sq(D_j);
                
                pointArray = pointsBeyondP2(this,v_j,stepSize);
                pointArray.pop();
                
                var sum1 = 0; var x; var r; var constantTermW2;

                for(var i =0; i<pointArray.length; i++){
                    x = pointArray[i];//phro_j(r)
                    r = distP2(x,v_j);
                    
                    p_j = particleShadows[neighborAgent].sensingModelFunction(x);
                    p_i = Math.exp(-1*this.sensingDecayFactor*distP2(this.position,x));
                    if(p_j>0){
                        // this is correct
                        ////constantTermW2 = getEventDensity(x)*(1-this.detectionProbability(x,true))*this.sensingModelFunctionForARay(x);
                        // this is wrong - try to avoid the following line and try to use the line above
                        constantTermW2 = getEventDensity(x)*(1-this.detectionProbabilityWithout(x,true,neighborAgent))*p_i*p_j;

                        sum1 = sum1 + constantTermW2*r*stepSize;
                    } 
                }
                
                derivativeXPart2 = derivativeXPart2 + factor1*sum1;
                derivativeYPart2 = derivativeYPart2 + factor2*sum1;
                
            }
        }
        //// end nonreflex vertices of agent  i (not neighbor j's)

        //// nonreflex vertices of neighbor j = 0
        // no need to do this as it is zero
        //// end nonreflex vertices of neighbor j

        // derivative Part 3 - arc integral : Notes on OneNote Coverage19
        var deltaTheta = stepSize/this.senRange;

        var arcSection = this.getArcSection(neighborAgent);

        var constantFactor = -1*deltaTheta*this.senRange; // R d\theta
        var pointArrayP = []
        var constantTerm = 0;

        if(arcSection.length==0){
            arcSections = [[-1*Math.PI, Math.PI]]; // do a full scan
            for(var j = 0; j<arcSections.length; j++){// considering each arc section
                
                var theta = arcSections[j][0] + (deltaTheta/2);
                while(Math.abs(theta-arcSection[1])>(deltaTheta/2)){
                    var interestedPoint = new Point2(this.position.x + (this.senRange-1)*cos(theta), this.position.y + (this.senRange-1)*sin(theta)); 
                    if(isLineOfSight(this.position,interestedPoint)){
                        p_i = this.sensingModelFunction(interestedPoint);
                        p_j = particleShadows[neighborAgent].sensingModelFunction(interestedPoint);
                            
                        if(p_i>0 && p_j>0){
                            pointArrayP.push(interestedPoint);
                            constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent))*p_i*p_j;
                            derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                            derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           
                        }        
                    }
                    theta = theta + deltaTheta;
                    if(theta>Math.PI){
                        theta = theta - 2*Math.PI;
                    }  
                }
            }
        }else{
            var theta = arcSection[0] + (deltaTheta/2);
            while(Math.abs(theta-arcSection[1])>(deltaTheta/2)){
                
                var interestedPoint = new Point2(this.position.x + (this.senRange-1)*cos(theta), this.position.y + (this.senRange-1)*sin(theta)); 
                
                p_i = this.sensingModelFunction(interestedPoint);
                p_j = particleShadows[neighborAgent].sensingModelFunction(interestedPoint);
                    
                if(p_i>0 && p_j>0){
                    pointArrayP.push(interestedPoint);
                    constantTerm = getEventDensity(interestedPoint)*(1-this.detectionProbabilityWithout(interestedPoint,true,neighborAgent))*p_i*p_j;
                    derivativeXPart3 = derivativeXPart3 + constantTerm*cos(theta);           
                    derivativeYPart3 = derivativeYPart3 + constantTerm*sin(theta);           
                }

                theta = theta + deltaTheta;
                if(theta>Math.PI){
                    theta = theta - 2*Math.PI;
                }     
            }
            /*noStroke();
            fill(100);
            printPointArrayP2(pointArrayP,"green",2);*/
        }

        derivativeXPart3 = constantFactor*derivativeXPart3;
        derivativeYPart3 = constantFactor*derivativeYPart3;
        // END derivative part 3 

        ////return ([derivativeXPart1+derivativeXPart2+derivativeXPart3,derivativeYPart1+derivativeYPart2+derivativeYPart3]);
        /////return ([derivativeXPart1+derivativeXPart2,derivativeYPart1+derivativeYPart2]);
        return ([derivativeXPart1,derivativeYPart1,derivativet]);
    }

    this.getArcSection = function(neighborAgent){// intersection between sensor ranges wrt s_i
        var s_i = this.position;
        var s_j = particleShadows[neighborAgent].position;
        var R = this.senRange;
        var D = distP2(s_i,s_j);
        
        
        if( D > 0 && D < 2*R){
            var d = sqrt(sq(R)-sq(D/2));
            var v = normalizeP2(minusP2(s_j,s_i));
            var n1 = new Point2(-1*v.y,v.x);
            var n2 = new Point2(v.y,-1*v.x);

            var p1 = plusP2(productP2(plusP2(s_i,s_j),0.5),productP2(n1,d));
            var p2 = plusP2(productP2(plusP2(s_i,s_j),0.5),productP2(n2,d));

            var theta1 = Math.atan2(p1.y-s_i.y,p1.x-s_i.x);
            var theta2 = Math.atan2(p2.y-s_i.y,p2.x-s_i.x);
            var thetaArray  = [theta1, theta2];
            thetaArray.sort(function(a, b){return a - b});
            theta1 = thetaArray[0];
            theta2 = thetaArray[1];
            
            var alpha = Math.atan2(s_j.y-s_i.y,s_j.x-s_i.x);
            if(theta1 < alpha && theta2 > alpha){
                return [theta1, theta2];
            }else if(theta1 < alpha && theta2 < alpha){
                return [theta2, theta1];
            }else if(theta1 > alpha && theta2 > alpha){
                return [theta2, theta1];
            }else{
                //print('Single arc finding error')
                return [theta1, theta2];
            }
         
        }
        else if(D > 2*R){
            return [];// not neighbors !!
        }
        else{
            return [-1*Math.PI, Math.PI];
        }
    }

    this.getArcSections = function(){

        var arcIntersectingPoints = findObstacleIntersectingPoints(this);
        var nearbyNonReflexVertices = this.getNearbyNonReflexVertices();
        
        for(var j = 0; j < nearbyNonReflexVertices.length; j = j+2){
            
            var v_j = obstacles[nearbyNonReflexVertices[j]].nonReflexVertices[nearbyNonReflexVertices[j+1]];
            
            var Vj = plusP2(this.position,productP2(normalizeP2(minusP2(v_j,this.position)),this.senRange));
            
            var normalDirection = normalizeP2(minusP2(Vj,this.position));
            var normalDirection1 = new Point2(-normalDirection.y,normalDirection.x);
            var normalDirection2 = new Point2(normalDirection.y,-normalDirection.x);
            var Vj1 = plusP2(Vj,productP2(normalDirection1,2));
            var Vj2 = plusP2(Vj,productP2(normalDirection2,2));
            if(((!isOutOfCanvas(Vj1)) && isLineOfSight(this.position,Vj1)) || ((!isOutOfCanvas(Vj2)) && isLineOfSight(this.position,Vj2))){ 
                arcIntersectingPoints.push(Vj);
            }
        }

        if(arcIntersectingPoints.length%2==1){
            //print("Arc finding error: "+arcIntersectingPoints.length);
            return [[],[],[]];// neglect all arcs
            //print(arcIntersectingPoints.length);
        }else if(arcIntersectingPoints.length==0){// no intersection whatsoever
            return [[-1*Math.PI, Math.PI],[],[]];
        }

        var arcAngles = []; 
        for(var j =0; j<arcIntersectingPoints.length; j++){
            arcAngles.push(Math.atan2(arcIntersectingPoints[j].y-this.position.y,arcIntersectingPoints[j].x-this.position.x));
            //ellipse(arcIntersectingPoints[j].x, arcIntersectingPoints[j].y, 10, 10);
        }

        arcAngles.sort(function(a, b){return a-b;});
        //print(arcAngles);
        arcIntersectingPoints =[];
        var stepSizeLinear = 2; //2 pixels
        var deltaTheta = stepSizeLinear/this.senRange;
        var pointBefore; var pointAfter;
        var anglePairs = [];
        var arcLines = [];
        for(var j=0; j<arcAngles.length; j++){
            //print(arcAngles[j])
            arcIntersectingPoints.push(new Point2(this.position.x+this.senRange*cos(arcAngles[j]),this.position.y+this.senRange*sin(arcAngles[j])));            
            pointBefore = new Point2(this.position.x+this.senRange*cos(arcAngles[j]-deltaTheta),this.position.y+this.senRange*sin(arcAngles[j]-deltaTheta));
            pointAfter = new Point2(this.position.x+this.senRange*cos(arcAngles[j]+deltaTheta),this.position.y+this.senRange*sin(arcAngles[j]+deltaTheta));

            if(isLineOfSight(this.position,pointBefore)&&!isOutOfCanvas(pointBefore) && j==0 ){
                anglePairs.push([-Math.PI,arcAngles[j]]);
                
                var pointBeforeT = new Point2(this.position.x+this.senRange*cos(-Math.PI),this.position.y+this.senRange*sin(-Math.PI));
                arcLines.push([pointBeforeT,arcIntersectingPoints[j]]);
            }else if(isLineOfSight(this.position,pointAfter)&&!isOutOfCanvas(pointAfter) && j<(arcAngles.length-1)){
                anglePairs.push([arcAngles[j],arcAngles[j+1]]);
                
                var pointAfterT = new Point2(this.position.x + this.senRange*cos(arcAngles[j+1]),this.position.y + this.senRange*sin(arcAngles[j+1]));
                arcLines.push([arcIntersectingPoints[j],pointAfterT]);
            }else if(isLineOfSight(this.position,pointAfter)&&!isOutOfCanvas(pointAfter) && j==(arcAngles.length-1)){
                anglePairs.push([arcAngles[j],Math.PI]);

                var pointAfterT = new Point2(this.position.x + this.senRange*cos(Math.PI),this.position.y + this.senRange*sin(math.PI));
                arcLines.push([arcIntersectingPoints[j],pointAfterT]);
            }

        }

        for(var j =0; j<arcLines.length; j++){
            ////line(arcLines[j][0].x,arcLines[j][0].y,arcLines[j][1].x,arcLines[j][1].y);
        }
        return [anglePairs,arcAngles,arcIntersectingPoints] ;
    }


    this.testConvergenceForNoBoosting = function(){// without boosting - (normal operation of coverage control is decentralized)

        var derivativeAbsSquareSum = this.derivativeAbsSquareSum;
        var derivativeAbsSquareSumThreshold = sq(descretizationLevel/2)/this.stepSizeSquareSum;
        var derivativeSumValue = this.derivativeSumValue;

        if(derivativeAbsSquareSum < derivativeAbsSquareSumThreshold && this.convergenceTestPeriodLength>4){
            // best convergence situation
            this.derivativeAbsSquareSum = 0;//resetting
            this.stepSizeSquareSum = 0;//resetting
            this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
            this.convergenceTestPeriodLength = 0; //reset /hat{K}

            this.localMinimaReached = true;
            
        }else if(this.oscillatingCount>100){
            // small oscillations situation
            if(this.oscillatingWidth<descretizationLevel){
                
                this.derivativeAbsSquareSum = 0;//resetting
                this.stepSizeSquareSum = 0;//resetting
                this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
                this.convergenceTestPeriodLength = 0; //reset /hat{K}
                
                this.localMinimaReached = true;

            }else{// large oscillations !!!
                this.localMinimaReached = false;
                //print('Large oscillations in '+this.id+'Tuning K1');
                this.tuneLipschitzConstant();
            }
        }
        else if(this.convergenceTestPeriodLength >= testJustConvergenceInterval){
            
            //print('resetted '+derivativeAbsSquareSum+' , '+derivativeAbsSquareSumThreshold+' , '+derivativeSumValue.lengthP2());
            this.derivativeAbsSquareSum = 0;//resetting
            this.stepSizeSquareSum = 0;//resetting
            this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
            this.convergenceTestPeriodLength = 0; //reset /hat{K}

            if(distP2(this.position,this.lastPositionChecked)<descretizationLevel/2){
                this.localMinimaReached = true;
            }

            this.lastPositionChecked = this.position;

        }else if(this.localMinimaReached && distP2(this.position,this.lastPositionChecked)>descretizationLevel){ // perturbation detection
            
            this.derivativeAbsSquareSum = 0;//resetting
            this.stepSizeSquareSum = 0;//resetting
            this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
            this.convergenceTestPeriodLength = 0; //reset /hat{K}

            this.localMinimaReached = false;
        }

        this.convergenceTestPeriodLength = this.convergenceTestPeriodLength + 1;
        // need to add a code to detect putrbations
    }

    this.testConvergenceForBoosting = function(){ 
        // more strict way to change the this.localMinimaReached

        var derivativeAbsSquareSum = this.derivativeAbsSquareSum;
        var derivativeAbsSquareSumThreshold = sq(descretizationLevel/2)/this.stepSizeSquareSum;
        var derivativeSumValue = this.derivativeSumValue;

        if(derivativeAbsSquareSum < derivativeAbsSquareSumThreshold && this.convergenceTestPeriodLength>4){
            // best convergence situation
            this.derivativeAbsSquareSum = 0;//resetting
            this.stepSizeSquareSum = 0;//resetting
            this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
            this.convergenceTestPeriodLength = 0; //reset /hat{K}

            this.localMinimaReached = true;
            
        }else if(this.oscillatingCount>100){
            // small oscillations situation
            if(this.oscillatingWidth<descretizationLevel){
                
                this.derivativeAbsSquareSum = 0;//resetting
                this.stepSizeSquareSum = 0;//resetting
                this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
                this.convergenceTestPeriodLength = 0; //reset /hat{K}
                
                this.localMinimaReached = true;

            }else{// large oscillations !!!
                this.localMinimaReached = false;
                //print('Large oscillations in '+this.id+'Tuning K1');
                this.tuneLipschitzConstant();
            }
        }
        else if(this.convergenceTestPeriodLength >= testAgentLocalMinimaReachedInterval){
            
            //print('Resetted '+derivativeAbsSquareSum+' , '+derivativeAbsSquareSumThreshold+' , '+derivativeSumValue.lengthP2());
            this.derivativeAbsSquareSum = 0;//resetting
            this.stepSizeSquareSum = 0;//resetting
            this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
            this.convergenceTestPeriodLength = 0; //reset /hat{K}

            if(distP2(this.position,this.lastPositionChecked)<descretizationLevel/2){
                this.localMinimaReached = true;
            }

            this.lastPositionChecked = this.position;

        }else if(this.localMinimaReached && distP2(this.position,this.lastPositionChecked)>descretizationLevel){ // perturbation detection
            
            this.derivativeAbsSquareSum = 0;//resetting
            this.stepSizeSquareSum = 0;//resetting
            this.derivativeSumValue = new Point2(0,0); //resetting - but this is not used 
            this.convergenceTestPeriodLength = 0; //reset /hat{K}

            this.localMinimaReached = false;
        }

        this.convergenceTestPeriodLength = this.convergenceTestPeriodLength + 1;
        // need to add a code to detect putrbations

    }

    this.updateDecentralizedBoostingProcess = function(){// when decentralized boosting is there
        // we do not need this kind of function for centralized boosting situation
        // because it is governed by central unit - function 'testLocalMinimaReached()' in simulation.js is enough

        //// resetting and finding the convergence
        this.testConvergenceForBoosting();
        //// end - resetting and finding the convergence

        var optimalityReachedShadowCount = 0;
        var neighborsNew;
        var neighborsOld; 
        var combinedCoverageNew;
        var combinedCoverageOld;

        // check for neighborhood changes
        var continueIteration = true;
        if(addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==1){
            //print("here")
            //var neighborsNew = this.getNeighbors();
            neighborsNew = this.getEffectiveNeighbors();
            neighborsOld = this.neighborsOld;
            combinedCoverageNew = this.getLocalCombinedCoverageLevel();
            combinedCoverageOld = this.combinedCoverageOld;

            if(isEqualArrays(neighborsOld,neighborsNew)){
                // all is fine; so far neighborhood has not changed
                this.combinedCoverageOld = combinedCoverageNew; 
                //if(this.optimalCoverageH1==0){print("here0");}
            }else{// neighbor set has been changed
                // has coverage improved during previous session? If yes, carry on boosting stage; else
                continueIteration = false;
                if(combinedCoverageOld > this.optimalCoverageH1){// boosting has improved the coverage untill last point
                    // need to carry on the boosting stage; make appropriate changes
                    this.optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
                    this.optimalCoverageS1 = this.position; //new staring point
                    this.neighborsOld = neighborsNew; // new neighbor set
                    this.combinedCoverageOld = combinedCoverageNew;
                    consolePrint("Agent "+(this.id+1)+"'s neighborhood changed while boosting; Coverage improved from boosting; Boosting continues");
                    //if(this.optimalCoverageH1==0){print("here1");}
                }else{
                    this.optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
                    this.optimalCoverageS1 = this.position; //new staring point
                    this.neighborsOld = neighborsNew; // new neighbor set
                    this.combinedCoverageOld = combinedCoverageNew;

                    if(boostingMethod!=7){
                        this.isBoostingActivated = 2;//stop boosting temporary
                        consolePrint("Agent "+(this.id+1)+"'s neighborhood changed while boosting; No coverage improvement from boosting; Boosting temporarily halted; Iteration: "+boostingIterationNumber);
                    }else{
                        consolePrint("Agent "+(this.id+1)+"'s neighborhood changed while VA boosting; Boosting continues under no improvements; Iteration: "+boostingIterationNumber);
                    }
                    
                    //if(this.optimalCoverageH1==0){print("here1.5");}
                }


            }
            
        }
        else if(addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated>=2 && this.isBoostingActivated<=8){
            //print("here")
            //var neighborsNew = this.getNeighbors();
            neighborsNew = this.getEffectiveNeighbors();
            neighborsOld = this.neighborsOld;
            combinedCoverageNew = this.getLocalCombinedCoverageLevel();
            combinedCoverageOld = this.combinedCoverageOld;

            if(isEqualArrays(neighborsOld,neighborsNew)){
                // all is fine; so far neighborhood has not changed
                this.combinedCoverageOld = combinedCoverageNew; 
            }else{// neighbor set has been changed - reactivate boosting
                continueIteration = false;
                if(combinedCoverageOld > this.optimalCoverageH1){ // waiting has improved the coverage during previous session (stay same)
                    this.optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
                    this.optimalCoverageS1 = this.position; //new staring point
                    this.neighborsOld = neighborsNew; // new neighbor set
                    this.combinedCoverageOld = combinedCoverageNew;

                    //this.isBoostingActivated = 0;//reset boosting
                    consolePrint("Agent "+(this.id+1)+"'s neighborhood changed while in normal mode; Coverage improved from normal mode; Normal mode continues");
                }else{// waiting has not incresed coverage
                    this.optimalCoverageH1 = combinedCoverageNew; //new staring coverage level
                    this.optimalCoverageS1 = this.position; //new staring point
                    this.neighborsOld = neighborsNew; // new neighbor set
                    this.combinedCoverageOld = combinedCoverageNew;
                    
                    consolePrint("Agent "+(this.id+1)+"'s neighborhood changed while in normal mode; No coverage improvement from normal mode; Normal mode continues");
                }
            }
        
        }
        // end check for neighborhood change



        //////if(continueIteration && distanceIncrements[i]<4 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==0 && millis()>3000){
        if(continueIteration && this.localMinimaReached && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==0){
            
            this.isBoostingActivated = 1; //for the first time boosting is turned ON
            
            if(this.boostingIterationNumber==0){
                this.optimalCoverageH1 = this.getLocalCombinedCoverageLevel(); // this is local info
                this.optimalCoverageS1 = this.position;
                //if(this.optimalCoverageH1==0){print("here2");}
            }
            // keep track of neighbors and their positions
            
            //this.neighborsOld = this.getNeighbors(); // list of neighbors when started
            this.neighborsOld = this.getEffectiveNeighbors(); // list of neighbors when started
            this.combinedCoverageOld = this.optimalCoverageH1; // this will be updated at each interval
            
            this.boostingIterationNumber++;

            consolePrint("Agent "+(this.id+1)+"'s boosting activated; Iteration: "+this.boostingIterationNumber);

            // for VA - Boosting
            if(boostingMethod==7){// lock on to a target vertex anchor
                var targetVertexAnchorFound = this.findTargetVertexAnchor();
                if(!targetVertexAnchorFound){
                    this.isBoostingActivated = 2; // switching off boosting immediately
                    consolePrint("Agent "+(this.id+1)+"'s boosting deactivated (No targets found!); switching to normal; Iteration: "+this.boostingIterationNumber);
                }
            }
            // end for VA - Boosting

            

        //////}else if(continueIteration && distanceIncrements[i]<4 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==1){
        }else if(continueIteration && this.localMinimaReached && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==1){
            
            this.isBoostingActivated = 2;//now switch back to normal procedure
            
            consolePrint("Agent "+(this.id+1)+"'s boosting deactivated; switching to normal; Iteration: "+this.boostingIterationNumber);
            
        }
        //////else if(continueIteration && distanceIncrements[i]<5 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==2){
        else if(continueIteration && this.localMinimaReached && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==2){
            
            var improvement = this.getLocalCombinedCoverageLevel()-this.optimalCoverageH1;
            // if(improvement>10000){
            //  print(i,this.getLocalCombinedCoverageLevel(),this.combinedCoverageOld,this.optimalCoverageH1);
            //  print(i,this.getNeighbors(),this.neighborsOld);
            // }
            if (improvement<=0){//objectiveValue<=optimalCoverageH1
                
                this.isBoostingActivated = 3;//stop boosting anymore start waiting period
                
                if(displayPhysicalAgentsMode){
                    consolePrint("Agent "+(this.id+1)+"'s initial boosting/normal session finished; No improvement; Iteration: "+this.boostingIterationNumber+"; Mobilizing Physical Agents...");
                    driveAgentsToTargets();

                }else{
                    consolePrint("Agent "+(this.id+1)+"'s initial boosting/normal session finished; No improvement; Iteration: "+this.boostingIterationNumber);
                    
                }

            }else{
                                
                consolePrint("Agent "+(this.id+1)+"'s initial boosting/normal session finished; Coverage improvement (of "+round(improvement)+") achieved; boosting resetted; Iteration: "+this.boostingIterationNumber);
                
                this.optimalCoverageH1 = this.getLocalCombinedCoverageLevel();
                this.optimalCoverageS1 = this.position;
                this.isBoostingActivated = 0;//start over

            }
            
        }       
        //////else if(continueIteration && distanceIncrements[i]<5 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated>=3 && this.isBoostingActivated<8){
        else if(continueIteration && this.localMinimaReached && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated>=3 && this.isBoostingActivated<8){
            // waiting period 
            this.isBoostingActivated = this.isBoostingActivated + 1;

        }
        //////else if(continueIteration && distanceIncrements[i]<5 && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==8){
        else if(continueIteration && this.localMinimaReached && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==8){
            var localCoverageLevelTemp = this.getLocalCombinedCoverageLevel(); 
            var improvement = localCoverageLevelTemp-this.optimalCoverageH1;
            if (improvement<=0){//objectiveValue<=optimalCoverageH1
                
                // no resetting 
                //resetParticleShadowPositions(agentPositions);
                ////this.position.x = this.optimalCoverageS1.x;
                ////this.position.y = this.optimalCoverageS1.y;
                ////this.position = this.optimalCoverageS1;

                // IF THIS IS A SINGLE AGENT, WE NEED TO DE SOMETHING !!!
                if(this.neighborsOld.length==0){
                    this.position.x = this.optimalCoverageS1.x;
                    this.position.y = this.optimalCoverageS1.y;
                    this.position = this.optimalCoverageS1;
                    consolePrint("Agent "+(this.id+1)+"'s boosting sequence finished; No improvement achieved from boosting; Returned to best solution; Iteration: "+this.boostingIterationNumber+"; Final Objective : "+round(this.getLocalCombinedCoverageLevel()*100)/100);
                }
                // end - IF THIS IS A SINGLE AGENT, WE NEED TO DE SOMETHING !!!

                this.isBoostingActivated = 9;//stop boosting anymore
                if(displayPhysicalAgentsMode){
                    consolePrint("Agent "+(this.id+1)+"'s boosting sequence finished; No improvement achieved from boosting; Iteration: "+this.boostingIterationNumber+"; Mobilizing Physical Agents...");
                    driveAgentsToTargets();
                }else if(this.neighborsOld.length!==0){
                    consolePrint("Agent "+(this.id+1)+"'s boosting sequence finished; No improvement achieved from boosting; Iteration: "+this.boostingIterationNumber+"; Final Objective : "+round(this.getLocalCombinedCoverageLevel()*100)/100);
                }

            }else{
                                
                consolePrint("Agent "+(this.id+1)+"'s boosting sequence resetted; Improvement (of "+round(improvement)+") achieved; Iteration: "+boostingIterationNumber);
                this.optimalCoverageH1 = localCoverageLevelTemp;
                this.optimalCoverageS1 = this.position;
                this.isBoostingActivated = 0;//start over

            }

        }
        else if(continueIteration && distP2(this.position,this.lastPositionChecked)>2*descretizationLevel && this.isBoostingActivated==9){
        //////else if(continueIteration && (derivativeSumValues[i]>10 || systemPurturbed) && this.isBoostingActivated==9){
            this.lastPositionChecked = this.position;

            this.isBoostingActivated = 0; //reactivate boosting
            this.boostingIterationNumber = 0;
            
            this.diminishingStepSizeModeInitiated = false;
            consolePrint("Agent "+(this.id+1)+"'s boosting reactivated due to motion");
        

        }
        else if(continueIteration && this.isBoostingActivated==9){
            // need to propagate infomation
            optimalityReachedShadowCount++;
        }else{

        }


        // keeping track of intermediate points that we visited while  single agent is in boosting - maybe better than our initial or final values
        if(continueIteration && this.neighborsOld.length==0 && !this.localMinimaReached && addObstacleMode==0 && isSimulationMode && !mouseIsPressed && this.isBoostingActivated==1){
            var localCoverageLevelTemp = this.getLocalCombinedCoverageLevel(); 
            var improvement = localCoverageLevelTemp-this.optimalCoverageH1;
            if(improvement>0){
                this.optimalCoverageH1 = localCoverageLevelTemp; // this is local info
                this.optimalCoverageS1 = this.position;
            }
        }
        // end - keeping track of intermediate points that we visited while in boosting - maybe better than our initial or final values

    }


    
}

function tuneLipschitzConstantsLocally(){
    //////if(variableStepSizeMode){
        for(var i =0; i<particleShadows.length; i++){
            particleShadows[i].tuneLipschitzConstant();
        }
    //////}
    // update global lischitz constant
    K1List = [lipschitzConstantK1];
    for(var i =0; i<particleShadows.length; i++){
            K1List.push(particleShadows[i].lipschitzConstantK1);
    }
    lipschitzConstantK1 = max(K1List);
}


function followTrajectory(){//update and show 
	particleID = 0;
	//print(millis());
	
	particles[particleID].timeInstant++;
	if(particles[particleID].timeInstant<particles[particleID].trajectory.length){
		nextPosition = particles[particleID].trajectory[particles[particleID].timeInstant];
		document.getElementById("simulationTimeDisplay").innerHTML = nf(particles[particleID].timeInstant/10, 2, 2)+" s";
		/*simulationTimeDisplay.html(nf(particles[particleID].timeInstant/10, 2, 2)+" s");*/
		//print(nf(particles[particleID].controlInputArray[particles[particleID].timeInstant]));
		/*if(particles[particleID].timeInstant==particles[particleID].trajectory.length-1){
			controlInputDisplay.html(nf(0,2,2)+" i + "+nf(0,2,2)+ "j");	
		}
		else{
			controlInputDisplay.html(nf(particles[particleID].u[particles[particleID].timeInstant].x,2,2)+" i + "+nf(particles[particleID].u[particles[particleID].timeInstant].y,2,2)+ "j");
		}*/
	}
	else{
		trajectoryFollowMode = 0;
		//trajectoryFollowButton.style("background-color", "grey");
		clearInterval(trjectoryFollowingInterval);
		particles[particleID].timeInstant= 0;
	}
	particles[particleID].x = nextPosition.x;
	particles[particleID].y = nextPosition.y;
                        
    //moving in to use point2 arrays
	particles[particleID].position = nextPosition;

	//plotting
	//randomizePlot(particles[particleID].timeInstant);
	
	
}




function followShadows(){//update and show 
	//particleID = 0;
	//print(millis());
	var countTemp = 0 ;
	for(var particleID=0; particleID<particles.length; particleID++){
		particles[particleID].timeInstant++;
		if(particles[particleID].timeInstant<particles[particleID].trajectory.length){
			nextPosition = particles[particleID].trajectory[particles[particleID].timeInstant];
			particles[particleID].x = nextPosition.x;
			particles[particleID].y = nextPosition.y;
			particles[particleID].position = nextPosition;
			document.getElementById("simulationTimeDisplay").innerHTML = nf(particles[particleID].timeInstant/10, 2, 2)+" s";
/*
			simulationTimeDisplay.html(nf(particles[0].timeInstant/10, 2, 2)+" s");*/
		}
		else{
			countTemp++;
			//particles[particleID].shadowsFollowingMode = 0;		
			//particles[particleID].timeInstant= 0;
		}
	
		
	}
	
	if(countTemp==particles.length){
		//trajectoryFollowButton.style("background-color", "grey");
		if(shadowsFollowingMode==1){
			shadowsFollowingMode=0;
			clearInterval(shadowsFollowingInterval);
			resetParticleHistory();
		}	
		for(var particleID=0; particleID<particles.length; particleID++){
			particleShadows[particleID].x = particles[particleID].x;
			particleShadows[particleID].y = particles[particleID].y;
			particleShadows[particleID].position = particles[particleID].position;
		}
		
	}
}







function getFirstIntersectingVertices(a,b){//very first line which intersects with AB

    // need to identify the 
    var trial = getIntersectingVerticesList(a,b);//if we have an obvious answer
    if(trial.length==1){
        return trial[0];
    }


    var stepSize = 2;
    var pointArray = getInteriorTrajectoryPointsP2(a,b,stepSize);
    
    pointArray.push(b);
    pointArray.push(plusP2(b,productP2(normalizeP2(minusP2(b,a)),2)));//going little deeper

    var firstPointInsideObstacle = b;
    var intersectingVerticesList=[];
    for (var j = 0; j<pointArray.length; j++){//foward search

        if(!isLineOfSight(a,pointArray[j])){
            firstPointInsideObstacle = pointArray[j];
            intersectingVerticesList = getIntersectingVerticesList(a,firstPointInsideObstacle);
            break;
        }else if(isOutOfCanvas(pointArray[j])){
            firstPointInsideObstacle = pointArray[j]
            return getIntersectingBoundaryEdges(a,firstPointInsideObstacle);
            //intersectingVerticesList.push();
            break;
        }
    }

    if(intersectingVerticesList.length==0){
        ////print("interior point identification failed");
        ////print(intersectingVerticesList);
        return [];
    }else{
        return  intersectingVerticesList[0];
    }
}

function getIntersectingVerticesList(a,b){//line AB intersecting with what? get the complete list
    
    var normalVector = new Point2(-(b.y-a.y),b.x-a.x);
    var offset = dotP2(a,normalVector);
    var result = [];
    //print(normalVector);
    
    for(var i = 0; i<obstacles.length; i++){
        
        for(var j = 0; j < obstacles[i].vertices.length; j++){
            
            var count2 = 0;//num of edges which does not intersect
            //bounding box
            if(a.x > obstacles[i].largestXArray[j] && b.x > obstacles[i].largestXArray[j]){count2++;}
            else if(a.y > obstacles[i].largestYArray[j] && b.y > obstacles[i].largestYArray[j]){count2++;}
            else if(a.x < obstacles[i].smallestXArray[j] && b.x < obstacles[i].smallestXArray[j]){count2++;}
            else if(a.y < obstacles[i].smallestYArray[j] && b.y < obstacles[i].smallestYArray[j]){count2++;}
            else{//projection check2: // rigorous check stage 3
                
                //projection 1
                var projections1 = [];
                projections1.push(dotP2(obstacles[i].vertices[j],normalVector)-offset);
                if(j < obstacles[i].vertices.length-1){
                    projections1.push(dotP2(obstacles[i].vertices[j+1],normalVector)-offset);
                }else{
                    projections1.push(dotP2(obstacles[i].vertices[0],normalVector)-offset);
                }

                //projection 2
                var projections2 = [];
                projections2.push(dotP2(a,obstacles[i].normalDirectionArray[j])-obstacles[i].offsetArray[j]);
                projections2.push(dotP2(b,obstacles[i].normalDirectionArray[j])-obstacles[i].offsetArray[j]);
                
                //print(projections);
                if(projections1[0]*projections1[1]>0 || projections2[0]*projections2[1]>0 ){count2++;}
            }
            
            if(count2==0){//intersection found with a edge (obs-i,edge-j) - stop
                if(j < obstacles[i].vertices.length-1){
                    result.push([obstacles[i].vertices[j],obstacles[i].vertices[j+1]]);
                }else{
                    result.push([obstacles[i].vertices[j],obstacles[i].vertices[0]]);
                }
                
                
            }
            
            
        }
    }
    
    return result;
    
}




function getIntersectingVertices(a,b){//line AB intersecting with what? 
	
	var normalVector = new Point2(-(b.y-a.y),b.x-a.x);
	var offset = dotP2(a,normalVector);
	var result = [];
	//print(normalVector);
	
	for(var i = 0; i<obstacles.length; i++){
		
		for(var j = 0; j < obstacles[i].vertices.length; j++){
			
			var count2 = 0;//num of edges which does not intersect
			//bounding box
			if(a.x > obstacles[i].largestXArray[j] && b.x > obstacles[i].largestXArray[j]){count2++;}
			else if(a.y > obstacles[i].largestYArray[j] && b.y > obstacles[i].largestYArray[j]){count2++;}
			else if(a.x < obstacles[i].smallestXArray[j] && b.x < obstacles[i].smallestXArray[j]){count2++;}
			else if(a.y < obstacles[i].smallestYArray[j] && b.y < obstacles[i].smallestYArray[j]){count2++;}
			else{//projection check2: // rigorous check stage 3
				
				//projection 1
				var projections1 = [];
				projections1.push(dotP2(obstacles[i].vertices[j],normalVector)-offset);
                if(j < obstacles[i].vertices.length-1){
                    projections1.push(dotP2(obstacles[i].vertices[j+1],normalVector)-offset);
				}else{
                    projections1.push(dotP2(obstacles[i].vertices[0],normalVector)-offset);
                }

				//projection 2
				var projections2 = [];
				projections2.push(dotP2(a,obstacles[i].normalDirectionArray[j])-obstacles[i].offsetArray[j]);
				projections2.push(dotP2(b,obstacles[i].normalDirectionArray[j])-obstacles[i].offsetArray[j]);
				
				//print(projections);
				if(projections1[0]*projections1[1]>0 || projections2[0]*projections2[1]>0 ){count2++;}
			}
			
			if(count2==0){//intersection found with a edge (obs-i,edge-j) - stop
                if(j < obstacles[i].vertices.length-1){
                    return [obstacles[i].vertices[j],obstacles[i].vertices[j+1]];
                }else{
                    return [obstacles[i].vertices[j],obstacles[i].vertices[0]];
                }
                
				
			}
			
			
		}
	}
	
	return [];
	
}


function  getIntersectingBoundaryEdges(a,b){//line AB intersecting with what?
    var normalVector = new Point2(-(b.y-a.y),b.x-a.x);
    var offset = dotP2(a,normalVector);
    var result = [0];

    for(var j = 0; j < boundaryObstacle.vertices.length ; j++){
        
        var count2 = 0;//num of edges which does not intersect
        
        //bounding box
        if(a.x > boundaryObstacle.largestXArray[j] && b.x > boundaryObstacle.largestXArray[j]){count2++;}
        else if(a.y > boundaryObstacle.largestYArray[j] && b.y > boundaryObstacle.largestYArray[j]){count2++;}
        else if(a.x < boundaryObstacle.smallestXArray[j] && b.x < boundaryObstacle.smallestXArray[j]){count2++;}
        else if(a.y < boundaryObstacle.smallestYArray[j] && b.y < boundaryObstacle.smallestYArray[j]){count2++;}
        else{//projection check2: // rigorous check stage 3
            
            //projection 1
            var projections1 = [];
            projections1.push(dotP2(boundaryObstacle.vertices[j],normalVector)-offset);
            if(j < boundaryObstacle.vertices.length-1){
                projections1.push(dotP2(boundaryObstacle.vertices[j+1],normalVector)-offset);
            }else{
                projections1.push(dotP2(boundaryObstacle.vertices[0],normalVector)-offset);
            }
            //projection 2
            var projections2 = [];
            projections2.push(dotP2(a,boundaryObstacle.normalDirectionArray[j])-boundaryObstacle.offsetArray[j]);
            projections2.push(dotP2(b,boundaryObstacle.normalDirectionArray[j])-boundaryObstacle.offsetArray[j]);
            
            //print(projections);
            if(projections1[0]*projections1[1]>0 || projections2[0]*projections2[1]>0 ){count2++;}
        }
        //print(count2);
        if(count2==0){//intersection found with a edge (obs-i,edge-j) - stop
            if(j < (boundaryObstacle.vertices.length-1)){
                return [boundaryObstacle.vertices[j],boundaryObstacle.vertices[j+1]];
            }else{
                return [boundaryObstacle.vertices[j],boundaryObstacle.vertices[0]];
            }
        }
        
        
    }
   //print("errrrr");
    return result;

}





function resetParticleHistory(){
	for(var particleID=0; particleID<particles.length; particleID++){
		var lastPosition = particles[particleID].trajectory[particles[particleID].timeInstant];
		if (typeof lastPosition !== 'undefined') {
			particles[particleID].x = lastPosition.x;
			particles[particleID].y = lastPosition.y;
			particles[particleID].position = lastPosition;
		}	
		else{
			//print("trouble")
		}

		particles[particleID].s = [];
		particles[particleID].v = [];
		particles[particleID].sRef = [];//smae as referencePointTrajectory
		particles[particleID].u = [];
		particles[particleID].trajectory = [];//same as s
		particles[particleID].timeInstant = 0;//simulation instant living
		particles[particleID].referencePointTrajectory = [];
		particles[particleID].wayPoints = [];
	
		//particles[particleID].show(particleID);
		
	}
}


function resetParticleHistory2(){
	resetParticleHistory();

}


function purturbShadows(){
	var purturbationMagnitude=50;
	
	if(obstacles.length==0){
		for (var i=0; i<particleShadows.length; i++){
			var randomDirection = 2*PI*Math.random();
			particleShadows[i].x = particleShadows[i].x + purturbationMagnitude*cos(randomDirection);
			particleShadows[i].y = particleShadows[i].y + purturbationMagnitude*sin(randomDirection);
			particleShadows[i].position = new Point2(particleShadows[i].x,particleShadows[i].y);
		}
		
	}else{
		
		
		
	}
}

function resetParticleShadowPositions(desiredPositions){
    if(desiredPositions.length == particleShadows.length){
        for(var i=0; i<particleShadows.length; i++){
            particleShadows[i].x = desiredPositions[i].x;
            particleShadows[i].y = desiredPositions[i].y;
            particleShadows[i].position = desiredPositions[i];
        }
    }else{
        print("Sizes does not match");
    }


}

function getModeOfAgents(neighbors){
    var modes = [];
    for(var i=0;i<neighbors.length;i++){
        if(particleShadows[neighbors[i]].isBoostingActivated==1){
            modes.push(1);
        }else{
            modes.push(0);
        }
    }
    return modes;

}


function setModeOfAgents(agents,modes){
    for(var i=0;i<agents.length;i++){
        agents[i].isBoostingActivated = modes[i];
    }
}


function classify(agents){
    // each agent's features are sensing range and densing decay factor
    var numOfClasses = 0;
    var classificationResult = []; // result would be classifie indexes of the agents (s.t. numOfClasses = results.length) 
    var classFeatures = []; // contains the sensing range and the decay value specific to each class; (s.t. numOfClasses = results.length) 

    for(var i = 0; i<agents.length; i++){

        // search for rach class features with current agent's features
        var classFound = false;
        for(var j = 0 ; j < classFeatures.length; j++){
            if(classFeatures[j][0]==agents[i].senRange && classFeatures[j][1]==agents[i].sensingDecayFactor && classFeatures[j][2]==agents[i].agentCostRatio){
                classFound = true;
                classificationResult[j].push(i);
            }
        }

        if(classFound==false){
            classificationResult[classificationResult.length] = [i];
            classFeatures[classFeatures.length] = [agents[i].senRange,agents[i].sensingDecayFactor,agents[i].agentCostRatio];
        }

    }

    return classificationResult;

}


function removeAllAgents(){
    var L = particleShadows.length;
    particleShadows = [];
    particles = [];

    for(var i = 0; i<L; i++){
        if(submodularityMode!=2){
            document.getElementById("particleSelectDropdown").remove(i+1);
            
        }
    }
    document.getElementById("particleSelectDropdown").selectedIndex = 0;
    

    
}


// P3

function switchtValues(){
    for(var i = 0; i<particleShadows.length; i++){
        //var agentGamma = particleShadows[i].sensingCapacity*particleShadows[i].agentCostRatio
        var nextT =  1 - particleShadows[i].nextTToBe;
        var tolence = 0.001;
        if(nextT > 1){
            nextT =  1;
        }
        else if (nextT < tolence){
            nextT = tolence;
        }
        particleShadows[i].t = nextT;
    }

}




// P3 - end