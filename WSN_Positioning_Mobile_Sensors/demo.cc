
/* TKT-2300 C/C++ Demo */

#include <iterator>
#include<math.h>
#include <iostream>
#include<algorithm>
#include <string>
#include <sstream>
#include <locale>
#include<vector>
#include<ctime>

#define PI 3.14159265

#include "WSNExerciseAPI.hh"

using namespace std;
const unsigned ITERATIONS = 10000;
string location="";

std::string Node_Grid_Location(std::pair< double, double > XY_cord, int node_Num);
std::pair< double, double > Mobile_Node_Precise_Pos(int node_Num);
double Min_Cord_Cntr (double Zbl[]);
double Max_Cord_Cntr (double Zbl[]);
int RSSI_to_Radius (int RSSI_cntr);
int neighbor_finder(int node_Num);
std::pair< double, double> Fixed_Node_Location(int node_Num);
double distance_calculator(double x1, double y1,double x2,double y2);
std::string alarm_sender(std::pair< double, double > XY_cord_Precise_ptnt, std::string s,int pt_nbr, int stf,int patient, std::string Patient_Timer, int Flag_cond_reslvd, std::string staff_email);



//int patient;
//int staff_one;
//int staff_two;

  
  int  main() {
    std::pair< double, double > XY;
	std::pair< double, double > XY_cord_Precise;
	std::pair< double, double > XY_cord_Precise_ptnt;
	std::pair< double, double > XY_cord_Precise_stf1;
	std::pair< double, double > XY_cord_Precise_stf2;
    int patient;
    int staff_one;
    int staff_two;
	int staff_nrst;
	int staff_nrst_nbr;
	int Flag_cond_reslvd=0;
	std::string message_Generated;

	double x1,y1;
	double x2,y2;
    double x3,y3;
    double d1,d2,d;
    
	
	std::string staff1_email = "md.s.rahman@student.tut.fi";
	std::string staff2_email = "gourab.datta@student.tut.fi";
	std::string staff_email = "";

    cout<<"Enter Patient's Mobile Node:";
    cin>>patient;
    cout<<"Enter staff one's mobile NOde:";
    cin>>staff_one;
    cout<<"Enter staff two's mobile NOde:";
    cin>>staff_two;
    
    cout<<"patient's fixed neighbor";
    int patient_nbr;
    patient_nbr=neighbor_finder(patient);
    cout<<" is "<<patient_nbr<<endl;
    /////location of fix node///

	// patient precise position;
	
	XY_cord_Precise_ptnt = Mobile_Node_Precise_Pos(patient);
	
    
    x1=(XY_cord_Precise_ptnt.first);
    y1=(XY_cord_Precise_ptnt.second);

	cout<<"\n"<<"Patient Precise XY :"<<x1<<"  "<<y1<<"\n";


	// staff1 precise position;
	
	XY_cord_Precise_stf1 = Mobile_Node_Precise_Pos(staff_one);
	
    
    x2=(XY_cord_Precise_stf1.first);
    y2=(XY_cord_Precise_stf1.second);

	cout<<"\n"<<"Staff1 Precise XY :"<<x2<<"  "<<y2<<"\n";

    
    // staff2 precise position;
	
	XY_cord_Precise_stf2 = Mobile_Node_Precise_Pos(staff_two);
	
    
    x3=(XY_cord_Precise_stf2.first);
    y3=(XY_cord_Precise_stf2.second);

	cout<<"\n"<<"Staff2 Precise XY :"<<x3<<"  "<<y3<<"\n";

    
    d1=distance_calculator(x1,y1,x2,y2);
    d2=distance_calculator(x1,y1,x3,y3);
	
	if (d1<d2)
	{
		d = d1;
		staff_nrst = staff_one;
		//staff_nrst_nbr = staff1_nbr;
		staff_email = staff1_email;
	}
	else 
	{
		d = d2;
		staff_nrst = staff_two;
		//staff_nrst_nbr = staff2_nbr;
		staff_email = staff2_email;
	}
    
    ////alar sending///
    
    WSNExerciseAPI api;
    
    for( int iter = 0; iter < ITERATIONS; iter++ ) {
      
      /* wait for next message */
      WSNExerciseAPI::DataPacket dpacket = api.waitForData();
      if(dpacket.type=="ACCELERATION" && dpacket.nodeid==patient)
	  {
	    if(dpacket.data[3]>=5){
	    // send alarm//
		Flag_cond_reslvd = 1;
		api.sendCommand(staff_nrst,6);
		std::string Patient_Timer = dpacket.formattedTime;
	    message_Generated = alarm_sender(XY_cord_Precise_ptnt,"fallen Person",patient_nbr,staff_nrst, patient,Patient_Timer,Flag_cond_reslvd,staff_email);
		std::cout<<"\n"<<message_Generated<<"\n";
		}
		break;
	  }
      else if( dpacket.type=="BUTTON_PRESS" && dpacket.nodeid==patient && dpacket.data[0] == 1)
	  {
	  //SEND ALARM//
	  Flag_cond_reslvd = 1;
	  api.sendCommand(staff_nrst,6);
	  std::string Patient_Timer = dpacket.formattedTime;
	  message_Generated = alarm_sender(XY_cord_Precise_ptnt,"BUTTON_PRESS",patient_nbr,staff_nrst, patient,Patient_Timer,Flag_cond_reslvd,staff_email);
	  std::cout<<"\n"<<message_Generated<<"\n";
	  break;
	  
	  }
	}
	
	for( int iter = 0; iter < ITERATIONS; iter++ ) {
		/* wait for next message */
		WSNExerciseAPI::DataPacket dpacket = api.waitForData();
		  if(dpacket.type=="BUTTON_PRESS" && dpacket.nodeid==staff_nrst && dpacket.data[0] == 1)
		  {
			if (staff_nrst == staff_one)
			{
				api.sendCommand(staff_two,6);
				api.sendCommand(staff_one,7);
				api.sendEmail( staff2_email,"Condition Happened",message_Generated);
			}
			else
			{
				api.sendCommand(staff_two,7);
				api.sendCommand(staff_one,6);
				api.sendEmail( staff1_email,"Condition Happened",message_Generated);
			}
			break;
			
		  }
	}

    // staff1 precise position;
	
	XY_cord_Precise_stf1 = Mobile_Node_Precise_Pos(staff_one);
	
    
    x2=(XY_cord_Precise_stf1.first);
    y2=(XY_cord_Precise_stf1.second);

	cout<<"\n"<<"Staff1 New Precise XY :"<<x2<<"  "<<y2<<"\n";

    
    // staff2 precise position;
	
	XY_cord_Precise_stf2 = Mobile_Node_Precise_Pos(staff_two);
	
    
    x3=(XY_cord_Precise_stf2.first);
    y3=(XY_cord_Precise_stf2.second);

	cout<<"\n"<<"Staff2 New Precise XY :"<<x3<<"  "<<y3<<"\n";

    
    d1=distance_calculator(x1,y1,x2,y2);
    d2=distance_calculator(x1,y1,x3,y3);
	
	for( int iter = 0; iter < ITERATIONS; iter++ ) {
		/* wait for next message */
		WSNExerciseAPI::DataPacket dpacket = api.waitForData();
		//if response is negative send mail to staff two.//
		if((d1<=1.00) || (d2<=1.00))
		  //reset the alarm//
		{    
			Flag_cond_reslvd =2;
			api.sendCommand(staff_one,7);
			api.sendCommand(staff_two,7);
			std::string Patient_Timer = dpacket.formattedTime;
			message_Generated = alarm_sender(XY_cord_Precise_ptnt,"0000",patient_nbr,staff_nrst, patient,Patient_Timer,Flag_cond_reslvd,staff_email);
		}
	}


return 0;
}
    
int neighbor_finder(int node_Num)
{
  WSNExerciseAPI api;
	//ITERATIONS
	int counter = 0;
	int Node_id_cntr [3] ={}; 
	int RSSI_cntr [3] ={};
	int Best_Node_id=0;
	int Best_RSSI=0;
	std::pair< double, double > XY_cord;
	std::string message;

	for( int iter = 0; iter < ITERATIONS; iter++ ) {
		
		/* wait for next message */
		WSNExerciseAPI::DataPacket dpacket = api.waitForData();
		if(dpacket.nodeid==node_Num && dpacket.type=="NEIGHBORS"){

		for( int i = 0; i < 6; i++ )
			{
				if(i%2==0){
					Node_id_cntr [i/2] = dpacket.data[i];
				}
				else{
					RSSI_cntr [i/2] = dpacket.data[i];
				}
			}
		break;

		}
		
	}
	
	for( int i = 0; i < 3; i++ )
			{
				if (RSSI_cntr [i]>=Best_RSSI)
				{
					Best_Node_id=Node_id_cntr [i];
					Best_RSSI=RSSI_cntr [i];
				}
			}
return Best_Node_id;
  
}

 
std::pair< double, double> Fixed_Node_Location(int node_Num)

{
  
  WSNExerciseAPI api;
  std::pair< double, double > XY_cord;
  XY_cord = api.getXY( node_Num );

  return XY_cord;

}

double distance_calculator(double x1, double y1,double x2,double y2)

{
  double d=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
  return d;
}
//int patient, std::string Patient_Timer)


std::string alarm_sender(std::pair< double, double > XY_cord_Precise_ptnt,std::string s,int pt_nbr, int stf,int patient, std::string Patient_Timer, int Flag_cond_reslvd, std::string staff_email)
{
  WSNExerciseAPI api;
  
  //std::pair< double, double > XY_cord_Fixed;
  std::string message;
  std::string message1;
  std::string message2;
  std::string message3;
  std:: stringstream ss1;
    
    
  //XY_cord_Fixed = Fixed_Node_Location(pt_nbr);
  message1 = Node_Grid_Location(XY_cord_Precise_ptnt, pt_nbr);
      
  ss1 << patient;
  message2=ss1.str();
  message = "Alarm in " + message1 +  " on node " + ss1.str() + " Due to " + s + " on " +  Patient_Timer;
  
  message3 = "Alarm in " + message1 +  " on node " + ss1.str() + "  Resolved.";
  
  if (Flag_cond_reslvd == 1)
  {
	  api.sendEmail( staff_email,"Condition Happened",message);
	  return message;
  }
  else
  {
	  api.sendEmail( staff_email,"Condition Resolved",message3);
	  return message3;
  }

}  
    
  
std::string Node_Grid_Location(std::pair< double, double > XY_cord, int node_Num)
{
  WSNExerciseAPI api;
  std::string floor_loc;
  int X_cord;
  int Y_cord;
  std::string message;

  std:: stringstream ss1;
  std:: stringstream ss2;
  std:: stringstream ss3;
  char Y_grid ='A';
  floor_loc = api.getFloor( node_Num );
  X_cord = int(XY_cord.first)/20;
  Y_cord = int(XY_cord.second)/20;
  Y_grid += Y_cord;
  ss1<<X_cord;
  ss2<<Y_grid;
  message = floor_loc + " - " + ss2.str() + ss1.str();
  //std::cout<< message;
  return message;
  
  
}

int RSSI_to_Radius (int RSSI_cntr)
{
	int radius_Tx;
	
	if (RSSI_cntr == 3)
	{
		radius_Tx = 5;
	}
	else if (RSSI_cntr == 2)
	{
		radius_Tx = 10;
	}
	else if (RSSI_cntr == 1)
	{
		radius_Tx = 15;
	}
	else if (RSSI_cntr == 0)
	{
		radius_Tx = 50;
	}
	else
	{
		radius_Tx = 100;
	}
	
	return radius_Tx;


}

double Max_Cord_Cntr (double Zbl[])
{
	double Max_Cord = 0.00;
	for( int i = 0; i < 3; i++ )
			{
				if (Zbl [i]>= Max_Cord)
				{
					Max_Cord = Zbl [i];
				}
			}
	return Max_Cord;
}

double Min_Cord_Cntr (double Zbl[])
{
	double Min_Cord = 99999.99;
	for( int i = 0; i < 3; i++ )
			{
				if (Zbl [i] <= Min_Cord)
				{
					Min_Cord = Zbl [i];
				}
			}
	return Min_Cord;
}

std::pair< double, double > Mobile_Node_Precise_Pos(int node_Num)
{
	WSNExerciseAPI api;
	//ITERATIONS
	int counter = 0;
	int Node_id_cntr [3] ={}; 
	int RSSI_cntr [3] ={};
	int Best_Node_id=0;
	int Best_RSSI=0;
	double X_cord [3] ={};
	double Y_cord [3] ={};
	int Rad [3] = {};
	double Xbl [3] = {};
	double Ybl [3] = {};
	double Xtr [3] = {};
	double Ytr [3] = {};
	double angle = PI/4;
	double max_Xbl;
	double max_Ybl;
	double min_Xtr;
	double min_Ytr;
	double X_cord_Precise;
	double Y_cord_Precise;
	std::pair< double, double > XY_cord;
	std::pair< double, double > XY_cord_Precise;
	
	//std::string message;
	

	for( int iter = 0; iter < ITERATIONS; iter++ ) {
		
		/* wait for next message */
		WSNExerciseAPI::DataPacket dpacket = api.waitForData();
		if(dpacket.nodeid==node_Num && dpacket.type=="NEIGHBORS"){
			std::cout<< "Neighbors found" << "\n";
			for( int i = 0; i < 6; i++ )
				{
					if(i%2==0){
						Node_id_cntr [i/2] = dpacket.data[i];
						std::cout<< "Node_id_cntr" <<dpacket.data[i] << "\n";
					}
					else{
						RSSI_cntr [i/2] = dpacket.data[i];
						std::cout<< "RSSI_cntr" <<dpacket.data[i] << "\n";
					}
				}
			break;
		}
	}
		
	for( int i = 0; i < 3; i++ )
			{
				XY_cord = Fixed_Node_Location(Node_id_cntr[i]);
				X_cord [i]= (XY_cord.first);
				Y_cord [i]= (XY_cord.second);
				Rad [i] = RSSI_to_Radius (RSSI_cntr[i]);
				Xbl [i] = X_cord [i] - Rad [i] * cos (angle);
				Xtr [i] = X_cord [i] + Rad [i] * cos (angle);
				Ybl [i] = Y_cord [i] - Rad [i] * sin (angle);
				Ytr [i] = Y_cord [i] + Rad [i] * sin (angle);
				std::cout<< "Rad [i] :" << Rad [i] << "\n";
				std::cout<< "Xbl [i] :" << Xbl [i] << "\n";
				std::cout<< "Ybl [i] :" << Ybl [i] << "\n";
				std::cout<< "Xtr [i] :" << Xtr [i] << "\n";
				std::cout<< "Ytr [i] :" << Ytr [i] << "\n";

	
			}
	

	//int Xbl [3] = {};
	//int Ybl [3] = {};
	//int Xtr [3] = {};
	//int Ytr [3] = {};
	
	max_Xbl = Max_Cord_Cntr (Xbl);
	std::cout<< "\n" <<"max_Xbl: " << max_Xbl<< "\n";
	max_Ybl = Max_Cord_Cntr (Ybl);
	std::cout<< "max_Ybl: " << max_Ybl<< "\n";
	min_Xtr = Min_Cord_Cntr (Xtr);
	std::cout<< "min_Xtr: " << min_Xtr<< "\n";
	min_Ytr = Min_Cord_Cntr (Ytr);
	std::cout<< "min_Ytr: " << min_Ytr<< "\n";

	X_cord_Precise = (max_Xbl+min_Xtr)/2;
	Y_cord_Precise = (max_Ybl+min_Ytr)/2;
	
	std::cout<< "X_cord_Precise: " << X_cord_Precise<< "\n";
	std::cout<< "Y_cord_Precise: " << Y_cord_Precise<< "\n";

	XY_cord_Precise = std::make_pair(X_cord_Precise, Y_cord_Precise);
	return XY_cord_Precise;
	
}
