/* TUTWSN C++ interface for TKT-2300 */
#ifndef WSNEXERCISEAPI_HH
#define WSNEXERCISEAPI_HH

#include <vector>
#include <string>
#include <map>
#include "TCPSocket.hh"

/* node commands */
const int NODE_COMMAND_LED_GREEN_ON = 0;
const int NODE_COMMAND_LED_GREEN_OFF = 1;
const int NODE_COMMAND_REBOOT = 2;
const int NODE_COMMAND_LED_RED_ON = 6;
const int NODE_COMMAND_LED_RED_OFF = 7;

 
/* TUTWSN C++ interface */
class WSNExerciseAPI {
   
 public:
 
   /* data structure for single data sample from node */
   struct DataPacket {
    
      int nodeid;
      std::string formattedTime;
      std::string type;
      std::vector< double > data;
      int unixtime;
      int delay;
      
   };
   

   
   /* Constructor. */
   WSNExerciseAPI();
   
   /* Destructor */
   ~WSNExerciseAPI();
   
   /* Method which waits for data from sensor network.
    * Blocks until data is available. */
   DataPacket waitForData();

   /* Method returns std::string containing floor of the
    * requested node. */
   std::string getFloor( int node );
   
   /* Method returns std::pair containing double precision x,y -coordinates 
    * of the requested node. */
   std::pair< double, double > getXY( int node );
   
   /* Method to send e-mail warnings, etc. */
   void sendEmail( std::string address, std::string subject, std::string message );

   /* Method to send SMS warnings, etc. */
   void sendSMS( std::string number, std::string message );

   /* Send command to single sensor network node */
   void sendCommand( int nodeid, int command );
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   /* Move along. nothing to see here. */
      
private:

   /* Authenticate the api */
   void authenticateAPI();

   /* get locations from csv-file into the local container */
   void getLocations();
   std::vector< std::vector< std::string > > readCSV( std::string filename );   
   
   /* parse data from messsage. data is returned in a vector. */
   std::vector< double > parse( std::string data );
   
   /* convert unixtime to human readable format. 
    * YYYY-MM-DD HH:MM:SS */
   std::string epochToTimestamp( int etime );
   
   /* convert from network byteorder to host byteorder */
   int ntohInt( std::string data );
   short int ntohShort( std::string data );
   
   /* convert from host byteorder to network byteorder */
   std::string htonInt( int data );
   std::string htonShort( short data );
      
   /* storage for floors and coordinates */
   std::map< int, std::string > floors_;
   std::map< int, std::pair< double, double > > coordinates_;

   /* TCP socket */
   TCPSocket tcpsocket_;
   
};



/* exception to be thrown if something goes wrong */
class WSNExerciseAPIexception: public std::exception {
   
 public:
   
   WSNExerciseAPIexception( std::string what ) : what_(what) {}
   virtual ~WSNExerciseAPIexception() throw() {}
   
   virtual const char* what() const throw() {
      return what_.c_str();
   }
   
   std::string what_;
   
};


#endif
