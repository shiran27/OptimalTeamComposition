function Obstacle() {
   
	this.x = [];
	this.y = [];
	this.pixlSize = 1;//obstacle draw resolution
	this.invalidNonReflexVertices = [];
    this.wayPoints = [];


	this.updateIndex = function(indexValue){//this will run in case of deleting an existing obstacle
		this.obstacleIndex = indexValue;
	}
	
	this.update = function(x,y){
    	this.x.push(round(x/this.pixlSize)*this.pixlSize);
        this.y.push(round(y/this.pixlSize)*this.pixlSize);
        
        if(this.x.length>1){//text display
        	//textNow = obstacleText.value();
        	var textNow = document.getElementById("obstacleCoordinatesDisplay").value;
            document.getElementById("obstacleCoordinatesDisplay").value = [textNow,this.x[this.x.length-1],this.y[this.y.length-1]];
        }
        else{
        	document.getElementById("obstacleCoordinatesDisplay").value = [this.x[this.x.length-1],this.y[this.y.length-1]];
        }
        
        //for future use
        this.textString = document.getElementById("obstacleCoordinatesDisplay").value;
        
        this.xMean=0;
    	this.yMean=0;
    	for(var i = 0; i<this.x.length; i++){
    		this.xMean = this.xMean+this.x[i]; this.yMean = this.yMean+this.y[i];	
    	}
    	this.xMean = this.xMean/this.x.length;
    	this.yMean = this.yMean/this.y.length;
    	
    	//index
    	if(this.x.length>0){
    		this.obstacleIndex = Number(document.getElementById("obstacleDropdown").selectedIndex);//index of the object
    	}
    	
    };
    
    this.drawBoarder = function(){
    	beginShape();
    	
    	for(var i = 0; i<this.x.length; i++){
    		stroke(obstacleColor);
            fill(obstacleColor);
    		vertex(this.x[i],this.y[i]);
    	}
    	endShape(CLOSE);
    	
    	stroke(50)
    	fill(255)
    	
    	if(this.x.length>1){
    	   text(this.obstacleIndex+1,this.xMean,this.yMean);
    	}


        if(this.wayPoints.length>0 && displayPhysicalAgentsMode){
            printPointArrayP2(this.wayPoints,"darkgreen",4);
        }

    };
    
    
        
    this.calculateReflexVertices = function(){

        //Point2 arrays
        var reflexVertices = []; 
        var nonReflexVertices = [];
        var vertices = [];
        
        var angles = [];//angle with x+ axis
        var gradients = [];//gradient of edges
        var insideAngles = [];//interior angles
    	
    	for(var i=0; i<this.x.length; i++){
    		
    		vertices[i] = new Point2(this.x[i],this.y[i]);
    		
    		if(i>0){
    			angles[i-1] = atan2P2(vertices[i-1], vertices[i]);
    			gradients[i-1] = Math.tan(angles[i-1]);
    			
    		}
    		if(i==this.x.length-1){
    			angles[i] = atan2P2(vertices[i],vertices[0]);
    			gradients[i] = Math.tan(angles[i]);
    		}
    		
    		if(i>1){
    			insideAngles[i-2]=(angles[i-2]-angles[i-1])*180/PI;
    		}
    		if(i==this.x.length-1){
    			insideAngles[i-1] = (angles[i-1]-angles[i])*180/PI;
    			insideAngles[i] = (angles[i]-angles[0])*180/PI;
    		}
    		
    		
    	}
    	
    	
    	insideAngles.unshift(insideAngles.pop());	//cycle
    	    	
    	
    	var reflexVerticesCount = 0;
        var nonReflexVertexIndexes = [];
    	for(var i=0; i <insideAngles.length; i++){
    		 
    		if(insideAngles[i]<-180){
    			insideAngles[i] = 360+insideAngles[i];
    		}
    		
    		if(insideAngles[i]<0 || insideAngles[i]>180){
    			reflexVertices.push(vertices[i]);
    			reflexVerticesCount += 1;
    		}
    		else{
                if(typeof(this.invalidNonReflexVertices)=="undefined"){
    			    nonReflexVertices.push(vertices[i]);
                    nonReflexVertexIndexes.push(i); 
                }
                else if(!this.invalidNonReflexVertices.includes(i)){
                    nonReflexVertices.push(vertices[i]);
                    nonReflexVertexIndexes.push(i);          
                }
                else{
                    print("vertices "+i+" cancelled");
                }
                //print(insideAngles[i]);   
    		}
    		
    	}
    	//print(insideAngles);
    	//print(nonReflexVertices);
    	this.nonReflexVertices = nonReflexVertices;
        this.nonReflexVertexIndexes = nonReflexVertexIndexes;
    	this.vertices = vertices;
    	
    	// for bounding box
    	this.largestX = max(this.x);
    	this.largestY = max(this.y);
    	this.smallestX = min(this.x);
    	this.smallestY = min(this.y);
    	this.isConvex = reflexVerticesCount == 0;
    	
    	// for bounding box stage 2:
    	
    	var normalDirectionArray = [];
    	var offsetArray = [];
    	var largestXArray = [];
    	var largestYArray = [];
    	var smallestXArray = [];
    	var smallestYArray = [];
		
    	for(var i=0; i<vertices.length; i++){
    		c = this.vertices[i];
            
            if(i==(vertices.length-1)){
                d = this.vertices[0]
            }else{
                d = this.vertices[i+1];
            }
    		
    		normalDirectionArray.push(new Point2(-(d.y-c.y),d.x-c.x));
    		offsetArray.push(dotP2(c,normalDirectionArray[normalDirectionArray.length-1]));
    		largestXArray.push(max([c.x,d.x]));
    		largestYArray.push(max([c.y,d.y]));
    		smallestXArray.push(min([c.x,d.x]));
    		smallestYArray.push(min([c.y,d.y]));
    	}
    	
    	this.normalDirectionArray = normalDirectionArray;
    	this.offsetArray = offsetArray;
    	this.largestXArray = largestXArray;
    	this.largestYArray = largestYArray;
    	this.smallestXArray = smallestXArray;
    	this.smallestYArray = smallestYArray;

        //print(normalDirectionArray);
        //print(nonReflexVertexIndexes);
    	//this.drawBoarder();


        // generating wayPoints
        var wayPoints = [];
        //var centroid = new Point2(this.xMean,this.yMean);
        for(var i = 0; i<nonReflexVertexIndexes.length; i++){
            var vector1 =normalizeP2(normalDirectionArray[nonReflexVertexIndexes[i]]);

            if((nonReflexVertexIndexes[i]-1)>=0){
                var vector2 = normalizeP2(normalDirectionArray[nonReflexVertexIndexes[i]-1]);
            }
            else{
                var vector2 = normalizeP2(normalDirectionArray[normalDirectionArray.length-1]);//last element
            }

            var unitVector = normalizeP2(plusP2(vector1,vector2));
            wayPoints.push(plusP2(nonReflexVertices[i],productP2(unitVector,20)));
        }
        
        this.wayPoints = wayPoints;
        
        //print(wayPointsList);

    }
}

function obstacleSetupLoad(obstacleSetupIndex){

    obstacles = [];
    document.getElementById("obstacleCoordinatesDisplay").value = "";
    var obstacleDropdown = document.getElementById("obstacleDropdown");
    for(var i = obstacleDropdown.options.length - 1 ; i >= 0 ; i--){
        document.getElementById("obstacleDropdown").remove(i);
    }



    if(obstacleSetupIndex==0){//blank
    
    }
    else if(obstacleSetupIndex==1){// general
        var obstacle1 = [125,150,125,600,200,600,200,150];
        loadDefaultObstacle(obstacle1,[1,2]);
        var obstacle2 = [300,75,300,175,325,275,375,275,375,75];
        loadDefaultObstacle(obstacle2);
        var obstacle3 = [325,350,325,450,425,450,425,350];
        loadDefaultObstacle(obstacle3);
        var obstacle4 = [450,100,450,175,525,175,525,100];
        loadDefaultObstacle(obstacle4);
    }

    else if(obstacleSetupIndex==2){//room
        var obstacle1 = [550,0,550,200,575,200,575,0];
        loadDefaultObstacle(obstacle1,[0,3]);
        var obstacle2 = [150,0,150,200,175,200,175,0];
        loadDefaultObstacle(obstacle2,[0,3]);
        var obstacle3 = [250,0,250,200,275,200,275,0];
        loadDefaultObstacle(obstacle3,[0,3]);
        var obstacle4 = [350,0,350,200,375,200,375,0];
        loadDefaultObstacle(obstacle4,[0,3]);
        var obstacle5 = [450,0,450,200,475,200,475,0];
        loadDefaultObstacle(obstacle5,[0,3]);

        var obstacle6 = [75,275,75,600,95,600,95,275];
        loadDefaultObstacle(obstacle6,[1,2]);
        var obstacle7 = [275,275,275,600,295,600,295,275];
        loadDefaultObstacle(obstacle7,[1,2]);
        var obstacle8 = [475,275,475,600,495,600,495,275];
        loadDefaultObstacle(obstacle8,[1,2]);
        var obstacle9 = [575,275,575,600,595,600,595,275];
        loadDefaultObstacle(obstacle9,[1,2]);

    }
    else if(obstacleSetupIndex==3){// maze
        var obstacle1 = [110,0,110,490,130,490,130,0];
        loadDefaultObstacle(obstacle1,[0,2,3]);
        var obstacle2 = [130,470,130,490,490,490,490,470];
        loadDefaultObstacle(obstacle2,[0,1,3]);
        var obstacle3 = [470,110,470,470,490,470,490,110];
        loadDefaultObstacle(obstacle3,[0,1,2]);
        var obstacle4 = [230,110,230,130,470,130,470,110];
        loadDefaultObstacle(obstacle4,[1,2,3]);
        var obstacle5 = [230,130,230,370,250,370,250,130];
        loadDefaultObstacle(obstacle5,[0,2,3]);

        var obstacle6 = [250,350,250,370,370,370,370,350];
        loadDefaultObstacle(obstacle6,[0,1,3]);
        var obstacle7 = [350,230,350,350,370,350,370,230];
        loadDefaultObstacle(obstacle7,[1,2]);
    }
    else if(obstacleSetupIndex==4){// narrow
        var obstacle1 = [100,50,100,550,150,550,150,50];    
        loadDefaultObstacle(obstacle1);
    }

    for(var i = 0; i < obstacles.length; i++){
        obstacles[i].calculateReflexVertices();
    }
}