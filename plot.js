
var plotLayout1 = {
		title: 'Coverage Cost Vs Iteration Number', 
		autosize: true,
	    width: 600,
	    height: 300,
	    xaxis: {
			title: 'Iteration Number', 
			showline: true,
			showgrid: true, 
			zeroline: true,
		}, 
		yaxis: {
			title: 'Coverage Cost', 
		    showline: true,
		    showgrid: true,
		    zeroline: true,
		}
	}

var plotLayout2 = {
	title: 'Coverage Cost Increment Vs Iteration Number', 
	autosize: true,
    width: 600,
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
	},
}


var plotLayout3 = {
  title: 'Global objective function with Respect to s_N',
  //scene: {camera: {eye: {x: 0.4, y: -0.4, z:1}}},
  autosize: false,
  width: 1000,
  height: 1000,
  xaxis: {
		title: 'X - Position: s_{Nx}', 
		//autorange: 'reversed',
		showline: true,
		showgrid: true, 
		zeroline: true,
	}, 
	yaxis: {
		title: 'X - Position: s_{Yx}', 
		//autorange: 'reversed',
	    showline: true,
	    showgrid: true,
	    zeroline: true,
	    
	},
	zaxis: {
		title: 'Global Cost: H(s)', 
		//autorange: 'reversed',
	    showline: true,
	    showgrid: true,
	    zeroline: true,
	},
	
  // margin: {
  //   l: 20,
  //   r: 20,
  //   b: 20,
  //   t: 20,
  // },
};





var updatePlotsInterval;

var numberOfAgents;

var updatePlotIterationNumber = 0;


function initiatePlotting(){

	var myPlot1 = document.getElementById('myPlot1');
	var myPlot2 = document.getElementById('myPlot2');
	var my3DPlot = document.getElementById('my3DPlot');
	//console.log(myPlot1);

	var data1s = localStorage.getItem('key1');
	var data2s = localStorage.getItem('key2');
	var data3s = localStorage.getItem('key4');
	var data1 = JSON.parse(data1s);
	var data2 = JSON.parse(data2s);
	var data3 = JSON.parse(data3s);
	console.log(data3);
	
	document.getElementById("plotUpdateIterationDisplay").innerHTML = updatePlotIterationNumber;

	Plotly.newPlot(myPlot1, data1, plotLayout1,{displayModeBar: false});
	Plotly.newPlot(myPlot2, data2, plotLayout2,{displayModeBar: false});
	////Plotly.newPlot(my3DPlot, data3, plotLayout3,{displayModeBar: false});
	Plotly.newPlot(my3DPlot, data3, plotLayout3);

	//derivative and beta plot - initial
	var numberOfAgentsString = localStorage.getItem('key3');
	numberOfAgents = JSON.parse(numberOfAgentsString);
	//console.log(numberOfAgents);


	constructSpaceForDerivativePlots();
	drawDerivativePlots();

	constructSpaceForBetaPlots();
	drawBetaPlots();

	constructSpaceForQPlots();
	drawQPlots();

	updatePlotsInterval = setInterval(updatePlots,100); //starts updating sequence 

	
}


function updatePlots(){

	updatePlotIterationNumber++;
	document.getElementById("plotUpdateIterationDisplay").innerHTML = updatePlotIterationNumber;

	//console.log("some");
	var myPlot1 = document.getElementById('myPlot1');
	var myPlot2 = document.getElementById('myPlot2');
	////var my3DPlot = document.getElementById('my3DPlot');
	//console.log(TESTER);

	var data1s = localStorage.getItem('key1');
	var data2s = localStorage.getItem('key2');
	////var data3s = localStorage.getItem('key4');
	var data1 = JSON.parse(data1s);
	var data2 = JSON.parse(data2s);
	////var data3 = JSON.parse(data3s);
	//console.log(data3);
	
	Plotly.newPlot(myPlot1, data1, plotLayout1,{displayModeBar: false});
	Plotly.newPlot(myPlot2, data2, plotLayout2,{displayModeBar: false});
	////Plotly.newPlot(my3DPlot, data3, plotLayout3,{displayModeBar: false});


	// updating derivative Plots
	var numberOfAgentsString = localStorage.getItem('key3');
	var newNumberOfAgents = JSON.parse(numberOfAgentsString);
	//console.log(newNumberOfAgents);

	if(newNumberOfAgents != numberOfAgents){
		console.log('changed');
		initiatePlotting()
		/*constructSpaceForDerivativePlots();
		constructSpaceForBetaPlots();*/

	}
	numberOfAgents = newNumberOfAgents;

	drawDerivativePlots();
	drawBetaPlots();
	drawQPlots();


}

function stopUpdatingPlots(){
	console.log("Updating Plots Halted");
	clearInterval(updatePlotsInterval);
}

function requestUpdateOfGlobalCostPlot(){
    var data = true;
    var datas = JSON.stringify(data);
    localStorage.setItem('switchKey1', datas);

}

function plotGlobalCost(){
	var my3DPlot = document.getElementById('my3DPlot');
	var data3s = localStorage.getItem('key4');
	var data3 = JSON.parse(data3s);
	console.log(data3);
	Plotly.newPlot(my3DPlot, data3, plotLayout3,{displayModeBar: false});
}

function constructSpaceForDerivativePlots(){
	//clearing old divs
	document.getElementById('derivativePlotsParent').innerHTML = "";

	// appending new divs

    var outerDiv;
    var innerDiv;

    for(var i = 0; i<numberOfAgents; i++){
        outerDiv = document.createElement('div');
        outerDiv.id = 'outerDivIDD' + i;
        outerDiv.className = 'col col-lg-12';
        
        innerDiv = document.createElement('div');
        innerDiv.id = 'myPlotD' + (1+i);
        //outerDiv.className = 'col col-lg-12';
        outerDiv.appendChild(innerDiv);
        document.getElementById('derivativePlotsParent').appendChild(outerDiv);
    }
    
    
}

function drawDerivativePlots(){
	for(var i=0; i<numberOfAgents; i++){

		var plotLayout = {
							title: 'X - Component of the Gradient of Agent '+(i+1)+' (d<sub>'+(i+1)+'x(k)</sub>) Vs Iteration Number', 
							autosize: true,
						    width: 600,
						    height: 300,
						    xaxis: {
								title: 'Iteration Number', 
								showline: true,
								showgrid: true, 
								zeroline: true,
							}, 
							yaxis: {
								title: 'Gradient<sub>x</sub>: d<sub>'+(i+1)+'x(k)</sub>', 
							    showline: true,
							    showgrid: true,
							    zeroline: true,
							}
						}


		var myPlot = document.getElementById('myPlotD'+(1+i));
		//console.log(myPlot);

		var datas = localStorage.getItem('keyD'+(1+i));

		var data = JSON.parse(datas);
		//console.log(data);

		////Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
		Plotly.newPlot(myPlot, data, plotLayout);
	}
}


function constructSpaceForBetaPlots(){
	//clearing old divs
	document.getElementById('betaPlotsParent').innerHTML = "";

	// appending new divs

    var outerDiv;
    var innerDiv;

    for(var i = 0; i<numberOfAgents; i++){
        outerDiv = document.createElement('div');
        outerDiv.id = 'outerDivIDB' + i;
        outerDiv.className = 'col col-lg-12';
        
        innerDiv = document.createElement('div');
        innerDiv.id = 'myPlotB' + (1+i);
        //outerDiv.className = 'col col-lg-12';
        outerDiv.appendChild(innerDiv);
        document.getElementById('betaPlotsParent').appendChild(outerDiv);
    }
    
    
}


function drawBetaPlots(){
	for(var i=0; i<numberOfAgents; i++){

		var plotLayout = {
							title: 'beta* of Agent '+(i+1)+' Vs Iteration Number', 
							autosize: true,
						    width: 600,
						    height: 300,
						    xaxis: {
								title: 'Iteration Number', 
								showline: true,
								showgrid: true, 
								zeroline: true,
							}, 
							yaxis: {
								title: 'beta<sub>'+(i+1)+'(k)</sub><sup>*</sup>',//'beta_{'+(i+1)+'}^*', 
							    showline: true,
							    showgrid: true,
							    zeroline: true,
							}
						}


		var myPlot = document.getElementById('myPlotB'+(1+i));
		//console.log(myPlot);

		var datas = localStorage.getItem('keyB'+(1+i));

		var data = JSON.parse(datas);
		//console.log(data);

		Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
	}
}




function constructSpaceForQPlots(){
	//clearing old divs
	document.getElementById('QPlotsParent').innerHTML = "";

	// appending new divs

    var outerDiv;
    var innerDiv;

    for(var i = 0; i<numberOfAgents; i++){
        outerDiv = document.createElement('div');
        outerDiv.id = 'outerDivIDQ' + i;
        outerDiv.className = 'col col-lg-12';
        
        innerDiv = document.createElement('div');
        innerDiv.id = 'myPlotQ' + (1+i);
        //outerDiv.className = 'col col-lg-12';
        outerDiv.appendChild(innerDiv);
        document.getElementById('QPlotsParent').appendChild(outerDiv);
    }
    
    
}


function drawQPlots(){
	for(var i=0; i<numberOfAgents; i++){

		var plotLayout = {
							title: 'Q of Agent '+(i+1)+' Vs Iteration Number', 
							autosize: true,
						    width: 600,
						    height: 300,
						    xaxis: {
								title: 'Iteration Number', 
								showline: true,
								showgrid: true, 
								zeroline: true,
							}, 
							yaxis: {
								title: 'Q<sub>'+(i+1)+'(k)</sub>',//'beta_{'+(i+1)+'}^*', 
							    showline: true,
							    showgrid: true,
							    zeroline: true,
							}
						}


		var myPlot = document.getElementById('myPlotQ'+(1+i));
		//console.log(myPlot);

		var datas = localStorage.getItem('keyQ'+(1+i));

		var data = JSON.parse(datas);
		//console.log(data);

		Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
	}
}


function refreshAll(){
    location.reload(true);
}