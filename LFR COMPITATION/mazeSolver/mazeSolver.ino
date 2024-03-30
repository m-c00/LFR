/*   
Maze
A maze is a tour puzzle in the form of a complex branching passage through which the solver must find a route. In everyday speech, both maze and labyrinth denote a complex and confusing series of pathways, but technically the maze is distinguished from the labyrinth, as the labyrinth has a single through-route with twists and turns but without branches, and is not designed to be as difficult to navigate.


Sample Maze
LSRB Algorithm
This LSRB algorithm can be simplified into these simple conditions:

If you can turn left then go ahead and turn left,
else if you can continue driving straight then drive straight,
else if you can turn right then turn right.
If you are at a dead end then turn around.

The expunction of LSRB is shown below:
L- Left
R- Right
S- Straight
B- Turning around (Back).
The robot has to make these decisions when at an intersection. An intersection is any point on the maze where you have the 
opportunity to turn. If the robot comes across an opportunity to turn and does not turn then this is considered going straight.
 Each move was taken at an intersection or when turning around has to be stored.

At first, we see the searching process of the LSRB Algorithms. the diagrams are the sample of the processing the LSRB.


Path = L
The first diagram denotes the starting point and the first decision based on the LSRB priority so the robot is choosing the left side. 
Second decision Value has been stored in the register. The first decision diagram is shown,


Path = LB
In the second diagrams is denoted the after processing of the first decision and the next decision is taken to the back 
side because there are no more options are available. The second decision diagram is shown,


Path = LBL
In the Third diagrams is denoted the after processing of the second decision and the next decision is taken
 to the left side because there are no more options are available and the second decision Value has been stored
  in the register. The third decision diagram is shown,


Path = LBLL
In the Fourth diagrams is denoted the after processing of the third decision and the next decision is taken to 
the left side because there are no more options are available and the fourth decision Value has been stored in
 the register. The fourth decision diagram is shown,


Path = LBLLB
In the fifth diagrams is denoted the after processing of the fourth decision and the next decision is taken to
 the back side because there are no more options are available and the fifth decision Value has been stored in
  the register. The fifth decision diagram is shown,


Path = LBLLBS
In the sixth diagrams is denoted the after processing of the fifth decision and the next decision is taken to
 the straight side because there are no more options are available and the sixth decision Value has been stored
  in the register. The sixth decision diagram is shown,

In the seventh diagrams is denoted the after processing of the sixth decision and the next decision is taken 
to the right side because there are no more options are available and the seventh decision Value has been
 stored in the register. The seventh decision diagram is shown below, After the seventh step the searching 
 processes will be finished and then finally the “LBLLBSR” value is stored into the register.

The second process is the travelling processes. This process is performing the robot to go the destination
 without any searching and using the shortest path.

The travelling is simplifying the register value using the following equation and shorted and follow the 
value of the register the equations are shown below,

LBR = B

LBS = R

RBL = B

SBL = R

SBS = B

LBL = S

The register value is shorted from using the equations and the final register value is “SRR”.diagram is shown below


Path = S → SR → SRS
*/

// Coding
// Variable Initialisation
// IR Sensor
#define leftCenterSensor   A2
#define leftNearSensor     A1
#define leftFarSensor      A0
#define rightCenterSensor  A3
#define rightNearSensor    A4
#define rightFarSensor     A5

// IR Sensor readings
int leftCenterReading;
int leftNearReading;
int leftFarReading;
int rightCenterReading;
int rightNearReading;
int rightFarReading;

int leftNudge;
int replaystage;
int rightNudge;

// Enable Pins
#define enl 9   // LEFT
#define enr 10  // RIGHT

// speed of motor
#define spd 150

#define leapTime 200

// Motors
#define leftMotor1  5
#define leftMotor2  2
#define rightMotor1 4
#define rightMotor2 3

//for storing path details
char path[30] = {};
int pathLength;
int readLength;

// loop() Function
// This functions runs over and over, and check accordingly.
void loop(){
	readSensors();
  	//if only the middle two sensors can see the black line
	if(leftFarReading<200 && rightFarReading<200 && (leftCenterReading>200 || rightCenterReading>200)) 
	{ 
		straight();                                                                                    
	}
  	//otherwise goes to the leftHandWall method
	else
	{                                                                                              
		leftHandWall();                                                                                   
	}
}


// Read Sensor Value

void readSensors()	//accepts input from sensors
{
	leftCenterReading  = analogRead(leftCenterSensor);
	leftNearReading    = analogRead(leftNearSensor);
	leftFarReading     = analogRead(leftFarSensor);
	rightCenterReading = analogRead(rightCenterSensor);
	rightNearReading   = analogRead(rightNearSensor);
	rightFarReading    = analogRead(rightFarSensor); 
}

// Left Hand Wall Algoritham




void leftHandWall(){
	if( leftFarReading<200 && rightFarReading<200)	//indicates either 3way intersection or end of maze
	{
		digitalWrite(leftMotor1, HIGH);
		digitalWrite(leftMotor2, LOW);
		digitalWrite(rightMotor1, HIGH);
		digitalWrite(rightMotor2, LOW);
		delay(leapTime);
		readSensors();

		if(leftFarReading<200 || rightFarReading<200)	//if it moves forward and still sees all black, maze is done
		{
	  		done();
		}
		if(leftFarReading>200 && rightFarReading>200)	//otherwise it turns left
		{ 
	  		turnLeft();
		}
	}
  
	if(leftFarReading<200)	//indicates that a left turn is possible
	{ 
		digitalWrite(leftMotor1, HIGH);
		digitalWrite(leftMotor2, LOW);
		digitalWrite(rightMotor1, HIGH);
		digitalWrite(rightMotor2, LOW);
		delay(leapTime);
		readSensors();
		if(leftFarReading>200 && rightFarReading>200)	//checks to make sure maze isn't done
		{
			turnLeft();
		}
		else
		{
			done();
		}
	}
   
	if(rightFarReading<200)	//indicates a right turn is possible
	{
		digitalWrite(leftMotor1, HIGH);
		digitalWrite(leftMotor2, LOW);
		digitalWrite(rightMotor1, HIGH);
		digitalWrite(rightMotor2, LOW);
		delay(10);	//CHANGED FROM 30 to 10
		readSensors();

		if(leftFarReading<200)	//left turn is also possible
		{
			delay(leapTime-30);
			readSensors();

			if(rightFarReading<200 && leftFarReading<200)//end of maze
			{
				done();
			}
			else	//make the left turn
			{
			turnLeft();
			return;
			}
		}
		delay(leapTime-30);
		readSensors();
		if(leftFarReading>200 && leftCenterReading>200 && rightCenterReading>200 && rightFarReading>200)
		{
			turnRight();
			return;
		}
		path[pathLength] = 'S';
		pathLength++;
		if(path[pathLength-2] == 'B')
		{
			shortPath();
		}
		straight();
	}
	readSensors();
	if(leftFarReading>200 && leftCenterReading>200 && rightCenterReading>200 
  	&& rightFarReading>200 && leftNearReading>200 && rightNearReading>200)
	{
		turnAround();
	}
}

// shortest path

void shortPath()
{
	int shortDone = 0;
	if(path[pathLength-3] == 'L' && path[pathLength - 1] == 'R')
	{
		pathLength -= 3;
		path[pathLength] = 'B';
		shortDone = 1;
	}
	if(path[pathLength-3] == 'L' && path[pathLength - 1] == 'S' && shortDone == 0)
	{
		pathLength -= 3;
		path[pathLength] = 'R';
		shortDone = 1;
	}
	if(path[pathLength-3] == 'R' && path[pathLength - 1] == 'L' && shortDone == 0)
	{
		pathLength-=3;
		path[pathLength] = 'B';
		shortDone=1;
	}
	if(path[pathLength-3] == 'S' && path[pathLength - 1] == 'L' && shortDone == 0)
	{
		pathLength -= 3;
		path[pathLength] = 'R';
		shortDone = 1;
	}
	if(path[pathLength-3] == 'S' && path[pathLength - 1] =='S' && shortDone == 0)
	{
		pathLength-=3;
		path[pathLength] = 'B';
		shortDone=1;
	}
	if(path[pathLength-3] == 'L' && path[pathLength - 1] =='L' && shortDone == 0)
	{
		pathLength -= 3;
		path[pathLength] = 'S';
		shortDone = 1;
	}
	path[pathLength+1] = 'D';
	path[pathLength+2] = 'D';
	pathLength++;
}


// you need to change the delay for better prrformence






