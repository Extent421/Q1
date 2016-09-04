
var socket;
var rawLog = "a";
var overallBest = 999;
var sessionBest = 999;
var sessionNumber = 1;
var doConnect = false;

function logTime(msg) {
	rawLog = rawLog + msg + '\n';

	//var logger = document.getElementById('time');
	//logger.appendChild(document.createTextNode(msg + '\n'));
	//logger.scrollTop = logger.scrollHeight;
	var splitTime = msg.split(",");
	var overallFlag = false;
	var sessionFlag = false;
	thisTime = parseFloat(splitTime[2]);
	thisLap = parseInt(splitTime[0].match(/(\d+)$/)[0], 10);
	
		if(thisLap > 0) {
			if (thisTime < overallBest) {
				overallFlag = true;
				overallBest = thisTime;
			};
			if (thisTime < sessionBest) {
				sessionFlag = true;
				sessionBest = thisTime;
			};
		}

	var bestText = "";
	if (overallFlag) {
		bestText = ", overall best,";
	} else if (sessionFlag) {
		bestText = ", session best,";
	}

	var lapTable = document.getElementById('lapTimes');
	var newRow = lapTable.insertRow(-1);
	var lapCell = newRow.insertCell(0);
	var timeCell = newRow.insertCell(1);
	lapCell.innerHTML = thisLap;
	timeCell.innerHTML = thisTime;
	document.getElementById('lapScroll').scrollTop = document.getElementById('lapScroll').scrollHeight;

	refreshTimesTable();

	if ('speechSynthesis' in window) {
		// Synthesis support. Make your web apps talk!
		var readout = "lap "+ splitTime[0]+ bestText + ", "+ splitTime[2];
		var msg = new SpeechSynthesisUtterance(readout);
		msg.voice = speechSynthesis.getVoices().filter(function(voice) { return voice.name == localStorage.getItem("defaultVoice"); })[0];
		window.speechSynthesis.speak(msg);
	}
}
function status(msg) {
      document.getElementById('status').textContent = msg;
}

   function connect() {
   doConnect = true;
     var host = location.hostname;
     if (host == "") { host =  "10.0.2.128" };

     socket = new WebSocket('ws://'+host+':81/');
     status('Connecting');
     socket.onopen = function (event) {
       status('Connected');
	   document.getElementById('connectButton').innerHTML = "Disconnect";
	   document.getElementById('connectButton').onclick = disconnect;
     };
     socket.onmessage = function (event) {
       //log('RCVD: ' + event.data);
         if(event.data.startsWith('a') ){ 
             gotSample(event.data.substr(1, event.data.length));
         } else if(event.data.startsWith('c') ){ 
             logTime(event.data.substr(1, event.data.length));
         } else if(event.data.startsWith('d') ){ 
             gotSetting(event.data.substr(1, event.data.length));
         }
     };
     socket.onclose = function (event) {
       status('Disconnected.');
	   document.getElementById('connectButton').innerHTML = "Connect";
	   document.getElementById('connectButton').onclick = connect;
	   if (doConnect) setTimeout(connect, 1000);

     };
   }
   function toggleSettings() {
		var div = document.getElementById("settingsPage");
		var display = div.style.display == "none" ? "block" : "none";
		div.style.display = display;
		if (display == "block") {
			getSettings();
		}
		div = document.getElementById("mainPage");
		display = display == "none" ? "block" : "none";
		div.style.display = display;

   }
   
   function openTab(evt, tabName) {
		var i;
		var pages = document.getElementsByClassName("tabPage");
		for (i = 0; i < pages.length; i++) {
			pages[i].style.display = "none"; 
		}
		
		var tabs = document.getElementsByClassName("tabLink");
		for (i = 0; i < tabs.length; i++) {
			tabs[i].className = tabs[i].className.replace("selectedTab", "");
		}
		
		evt.currentTarget.className += " selectedTab";
		document.getElementById(tabName).style.display = "block";

		if (tabName == "settingsPage") {
			getSettings();
		}
		if (tabName == "statusPage") {
			getStatus();
			start();
		} else {
			stop();
		}

		
   }
   
   function pageLoaded() {
		document.getElementById("emailAddress").value = localStorage.getItem("emailAddress");
		

   }
  function emailChange() {
		localStorage.setItem("emailAddress",document.getElementById("emailAddress").value);
   } 
  function changeVoiceOption() {
		localStorage.setItem("defaultVoice",document.getElementById("voiceOption").value);
   } 
   function disconnect() {
	   doConnect = false;
     if (socket) {
       status('Disconnecting.');
       socket.close();
     }
   }
	function start() {
	    if (socket) {
	        socket.send('start');
	    }
	}
	function stop() {
	    if (socket) {
	        socket.send('stop');
	    } 
	}	   
   function sendCalLow() {
	     if (socket) {
	         socket.send('callo');
	     }
	   }
   function sendCalThresh() {
	     if (socket) {
	         socket.send('calthresh');
	     }
	   }
  function sendCalHi() {
	 if (socket) {
		 socket.send('calhi');
	 } 
   }
	function sessionStart() {
		if (socket) {
			socket.send('sessionStart');
			var node = document.getElementById('lapTimes');
			while (node.firstChild) {
				node.removeChild(node.firstChild);
			}
			
			rawLog = "lap, timecode, laptime, trigger length, trigger offset\n";
			sessionBest = 999;
			
			window.onbeforeunload = function() {
			    return "Reloading will erase current session.";
		};  

		}
	}
	function sessionEnd() {
	    if (socket) {
	        socket.send('sessionEnd');
	    }
	}
	function sendRestart() {
	    if (socket) {
	        socket.send('reboot');
	    }
	}	
	function getSettings() {
	    if (socket) {
	    	socket.send('getSettings' );
	    }
	}
	function getStatus() {
	    if (socket) {
	    	socket.send('getStatus' );
	    }
	}
	function setChannel() {
	    if (socket) {
	    	socket.send('sch'+ document.getElementById("channelNumber").value );
	    }
	}
	function minPulseChange() {
	    if (socket) {
	    	socket.send('smp'+ document.getElementById("minPulse").value );
	    }
	}
	function APSSIDChange() {
	    if (socket) {
	    	socket.send('sapn'+ document.getElementById("APSSID").value );
	    }
	}
	function APPasswordChange() {
	    if (socket) {
	    	socket.send('sapp'+ document.getElementById("APPassword").value );
	    }
	}
	function StationSSIDChange() {
	    if (socket) {
	    	socket.send('sstn'+ document.getElementById("stationSSID").value );
	    }
	}
	function StationPasswordChange() {
	    if (socket) {
	    	socket.send('sstp'+ document.getElementById("stationPassword").value );
	    }
	}
	function APModeChange() {
	    if (socket) {
			if (document.getElementById('APMode').checked ) {
		    	socket.send('sapm1' );
			} else {
		    	socket.send('sapm0' );
			}
	    }
	}
	function mailResult() {
		window.open('mailto:'+document.getElementById("emailAddress").value+'?subject=Times&body='+encodeURIComponent(rawLog));
	}
	
	function gotSample(data) {
		 document.getElementById('readingValue').textContent = data;
	}
	function gotSetting(data) {
		var dSplit = data.split(":");
		if (dSplit.length != 2) return;
		switch(dSplit[0]) {
			case "freq":
				document.getElementById('channelNumber').value = dSplit[1];
				break;
			case "minPulse":
				document.getElementById('minPulse').value = dSplit[1];
				break;
			case "stationSSID":
				document.getElementById('stationSSID').value = dSplit[1];
				break;
			case "stationPass":
				document.getElementById('stationPassword').value = dSplit[1];
				break;
			case "APSSID":
				document.getElementById('APSSID').value = dSplit[1];
				break;
			case "APPass":
				document.getElementById('APPassword').value = dSplit[1];
				break;
			case "APMode":
				if ( dSplit[1] == "1" ) {
					document.getElementById('APMode').checked = true;

				} else {
					document.getElementById('APMode').checked = false;
				}
				break;
			case "APstatus":
				document.getElementById('wifiStatusValue').textContent = dSplit[1];
				break;
			case "APIP":
				document.getElementById('wifiIPValue').textContent = dSplit[1];
				break;
			case "minCal":
				document.getElementById('loCalibrateValue').textContent = dSplit[1];
				break;
			case "maxCal":
				document.getElementById('hiCalibrateValue').textContent = dSplit[1];
				break;
			case "threshCal":
				document.getElementById('midCalibrateValue').textContent = dSplit[1];
				break;
			case "sessionRunning":
				console.log(data);
				if ( dSplit[1] == "1" ) {
					document.getElementById('sessionValue').textContent = "Running";
				} else {
					document.getElementById('sessionValue').textContent = "Stopped";
				}
				
				break;
				

				
			default:
				break;
		
		}
		
		 //document.getElementById('APSSID').value = dSplit[1];
	}	
   function update() {
     if (socket) {
       document.getElementById('readyState').textContent = socket.readyState;
     } else {
       document.getElementById('readyState').textContent = '-';
     }
   }
   function refreshTimesTable() {
	   var table = document.getElementById('lapTimes');
	   for (var i = 0, row; row = table.rows[i]; i++) {
		   //iterate through rows
		   //rows would be accessed using the "row" variable assigned in the for loop
			var timeValue = parseInt(row.cells[1].innerHTML) ;
			if (parseInt(sessionBest) == timeValue){
				row.className = "best";
			} else if (parseInt(sessionBest)+1 == timeValue) {
				row.className = "pace";
			} else {
				row.className = "";
			}
			   
		}
   }
   
   
window.speechSynthesis.onvoiceschanged = function() {
		var voices = speechSynthesis.getVoices();
		voices.forEach(function (voice, i) {
			var option = document.createElement('option');
			option.value = voice.name;
			option.innerHTML = voice.name;
			document.getElementById("voiceOption").appendChild(option);
		});
		document.getElementById("voiceOption").value = localStorage.getItem("defaultVoice");
}; 
   setInterval(update, 10);