/**
  * \file oroclient.cpp
  * \brief This file is the starter point of the "oroclient" executable.
  *        The SIGINT signal is overwritten in order to close properly the socket
  *        First, the TCP/IP socket with OROServer is opened.
  *        When oroserver-client is fully functionnal launch the ROS client-server and subscribe on "oroChatter"
  *        The ROS callback is implemented
  * \author DUMONT Emmanuel
  * \date 01/2016
  */



// Basic Lib
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/time.h>

#include <signal.h>         // Handling ctrl+C signal to close everything properly
#include <unistd.h>         // Manages sleep() close()

#include <time.h>           // Manage timeout
#include <stdint.h>         // Manage the packet_t

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>          // gethostbyname

// ROS lib
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <orolib/orolib.hpp>

// Inner lib
#include "lib/msgParser.h"


int gOtherClarify;


////////////////////////////////////////////
//                                        //
//         Function Declaration           //
//                                        //
////////////////////////////////////////////

/**
  * \fn void sigint_handler(int dummy)
  * \param[in] dummy Integer wich value corresponds to the signal asked
  *
  * \brief This function overwrite the action does when a SIGINT (ctrl+C) command is received by doing :
  *        Close the socket with OROServer if openned
  */
  
void sigint_handler(int dummy);
/**
  * \fn void oroClientCallback(const std_msgs::String::ConstPtr& msg)
  * \param[in] msg ROS message received
  *
  * \brief This function is a callback. the program wait until a message on "orochatter" is received.
  *        From this function, a parser is called in order to manage this message
  */
void oroClientCallback(const std_msgs::String::ConstPtr& msg);

/**
  * \fn void oroClientCallback(const std_msgs::String::ConstPtr& msg)
  * \param[in] msg ROS message received
  *
  * \brief This function is a callback. the program wait until a message on "orochatter" is received.
  *        From this function, a parser is called in order to manage this message
  */
void oroClientSendBack(char * buff2Send);


////////////////////////////////////////////
//                                        //
//                Main                    //
//                                        //
////////////////////////////////////////////



/**
  * \fn int main(int argc, char ** argv)
  * \param[in] argc Number of argument passed to the main function (at least 1, the command to launch the program)
  * \param[in] argv C String pointer. It contains the argument, stored as chains of char, passed to the program
  * \return EXIT_SUCCESS if OK, EXIT_FAILURE if NOK
  * 
  * \brief Main function of the OroClient program
  */
int main(int argc, char ** argv)
{
    ROS_INFO("\n\n\t-- Launching client --\n");
    
    // Resources declaration
    struct hostent *hostinfo = NULL;    // Get host info
    const char *hostname = "Client";    // Host name
    char address [32];                  // Ethernet address of the server
    const char * ptrAdd = address;      // Const char * used for the hostinfo

    int ret = 0;                        // Return value for the select
    struct timeval tv;                  // Select Timeout
       
    struct sockaddr_in self;            // Host server info
    char rcvBuffer[SIZE_BUFF];          // Received buffer
    
    
    struct timeval sTv;                 // Select timeout for accepting connection
    int retval = 0;                     // Returned value in the select
    fd_set rfds;                        // Set to watchout for select
    
    gOtherClarify = 0;                  // Initialize the global variable for clarification request
    
    gFindRequest = false;               // Initialize the global variable for find request
	  
	  // Create streaming socket
    if ( (gSocketInfo.otherSocket = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
	  {
		  perror("Socket");
		  exit(errno);
	  }

	  // Initialize address/port structure "clarification" socket
	  bzero(&self, sizeof(self));
	  self.sin_family = AF_INET;
	  self.sin_port = htons(OTHER_PORT);
	  self.sin_addr.s_addr = INADDR_ANY;

	  // Assign a port number to the "clarification" socket
      if ( bind(gSocketInfo.otherSocket, (struct sockaddr*)&self, sizeof(self)) != 0 )
	  {
		  perror("socket--bind");
		  exit(errno);
	  }

	  // Make it a "listening socket" "clarification" socket
	  if ( listen(gSocketInfo.otherSocket, 20) != 0 )
	  {
		  perror("socket--listen");
		  exit(errno);
	  }
	
    memset(rcvBuffer, 0, SIZE_BUFF);
	  struct sockaddr_in client_addr;
	  socklen_t addrlen = sizeof(client_addr);

    // Clear buffer
    memset(address,'\0',32);
    gSocketInfo.socket = 0;
    gSocketInfo.sinSize = 0;
    
    ROS_INFO("- Init ROS");
    
    // You must call one of the versions of ros::init() before using any other part of the ROS system.
    ros::init(argc, argv, "oroclient");
    
    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle n;
    
    // Override the signal interrupt from ROS
    signal(SIGINT, sigint_handler);

    ROS_INFO("- Creating the socket");


    // Socket creation (tcp)
    gSocketInfo.socket = socket(AF_INET, SOCK_STREAM, 0);
    if(gSocketInfo.socket == INVALID_SOCKET)
    {
        ROS_INFO("\t- Invalid socket");
        exit(EXIT_FAILURE);
    }

    ROS_INFO("- Server connection");

    snprintf(address, 32,"%s",ORO_ADDRESS);
    ROS_INFO("\t- Oro's IP address is '%s'",ptrAdd);

    // Get information from host
    hostinfo = gethostbyname(ptrAdd);
    if (hostinfo == NULL)
    {
        ROS_INFO ("\t- Unknown host '%s'", hostname);

        closesocket(gSocketInfo.socket);
        exit(EXIT_FAILURE);
    }
    
    // Add information to the structure
    gSocketInfo.sin.sin_addr    = *(IN_ADDR *) hostinfo->h_addr;
    gSocketInfo.sin.sin_port    = htons(ORO_PORT);
    gSocketInfo.sin.sin_family  = AF_INET;
    
    // Connect to the TCP-IP oro server
    if(connect(gSocketInfo.socket,(SOCKADDR *) &gSocketInfo.sin, sizeof(SOCKADDR)) == SOCKET_ERROR)
    {
        ROS_INFO ("\t- Error connect()");

        closesocket(gSocketInfo.socket);
        exit(EXIT_FAILURE);
    }
    
    // Update data
    gSocketInfo.sinSize = sizeof(gSocketInfo.sin);
        
    ROS_INFO("- Connected to OroServer\n- Subscribe to \"oroChatter\"");    
    
    
    // When oroserver-client is fully functionnal launch the ROS client-server
    // The subscribe() call is how you tell ROS that you want to receive messages on a given topic.
    ros::Subscriber sub = n.subscribe("oroChatter", 1000, oroClientCallback);
    
    ros::Rate r(1); // r.sleep next
    std::vector<int> connected;
    
    // Spin is done after
    int max = gSocketInfo.otherSocket+1;
    do
    {
      
      max = gSocketInfo.otherSocket+1;

      // Create a set to watch for select
      FD_ZERO(&rfds);
      FD_SET(gSocketInfo.otherSocket, &rfds);
      
      // Wait up to one second. (select accept ouside connection)
      sTv.tv_sec = 1;   sTv.tv_usec = 0;
      
      // Check if received message from other system
      retval = select(max, &rfds, NULL, NULL, &sTv);

      if (retval == -1)
      {
        ROS_INFO("- ERROR: select()");
        exit(EXIT_FAILURE);
      }
      else if (retval == 0){
        ROS_INFO("- No data within One second");
      }
      else // Accept a connection (creating a data pipe)
      {
        ROS_INFO("- Accepting Outside communication");
        gSocketInfo.clientfd = accept(gSocketInfo.otherSocket, (struct sockaddr*)&client_addr, (socklen_t*) &addrlen);
        ROS_INFO("- %s:%d connected\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // Once publish on OroChatter
        int sizeBuff = recv(gSocketInfo.clientfd, rcvBuffer, SIZE_BUFF, 0);

        ROS_INFO("- Received buffer from outside --%s-%d-", rcvBuffer, sizeBuff);
        
        
        ros::NodeHandle n;
        ros::Publisher oroChatter_pub = n.advertise<std_msgs::String>("oroChatter", 1000);
        usleep(300000); // Necessary to initialize the ros system

        ros::Rate loop_rate(10);
        std_msgs::String msg;

        std::stringstream ss;
        std::string s(rcvBuffer);

        char enumCmd = 0;
        enumCmd = (char)CMD_FIND;
        ss << "BigBrother#"<<enumCmd<<s;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        
        // Indicate that this is an outside request
        gOtherClarify += 1;
        
        // Send the command to the system
        oroChatter_pub.publish(msg);
        loop_rate.sleep();
      }
    
      ROS_INFO("- Machin false");
      // Check if received message from local node on oroChatter
      ros::spinOnce();
      r.sleep();
      
      
    }while(retval != -1); // While no errors on select


    ROS_INFO("- Close the socket\n");
    closesocket(gSocketInfo.socket);
    gSocketInfo.socket = 0;
    
    ret = system("clear");
    
    
    ROS_INFO("\n\n\t\t-- Close client --\n\nBye !\n\n");
    return EXIT_SUCCESS;
}


////////////////////////////////////////////
//                                        //
//            Other functions             //
//                                        //
////////////////////////////////////////////

/**
  * \fn void sigint_handler(int dummy)
  * \param[in] dummy Integer wich value corresponds to the signal asked
  *
  * \brief This function overwrite the action does when a SIGINT (ctrl+C) command is received by doing :
  *        Close the socket with OROServer if openned
  */
void sigint_handler(int dummy)
{
    ROS_INFO("- Oro_client is shutting down...");
        
    // If the socket is opened, close it
    if(gSocketInfo.socket != 0)
    {
        ROS_INFO("\t- Closing socket... ");
        closesocket(gSocketInfo.socket);
        gSocketInfo.socket = 0;
        gSocketInfo.otherSocket = 0;
    
        ROS_INFO(" Done !\n");
    }
    
    ROS_INFO("\n\n... Bye bye !\n   -Manu\n");
    exit(EXIT_SUCCESS); // Shut down the program
}


/**
  * \fn void oroClientCallback(const std_msgs::String::ConstPtr& msg)
  * \param[in] msg ROS message received
  *
  * \brief This function is a callback. the program wait until a message on "orochatter" is received.
  *        From this function, a parser is called in order to manage this message
  */
void oroClientCallback(const std_msgs::String::ConstPtr& msg)
{
  int retVal = 0; // Return value of function
  char * buff2TR = NULL;      // Buffer to send TO FREE AT THE END !
  char cpBuff2TR[SIZE_BUFF];
  
  ROS_INFO("- Received packet: --%s--", msg->data.c_str());
  
  // Parse the message and manage what to do
  buff2TR = parserMsgFromClient(msg);
  
  // Copy the buff2TR
  memset(cpBuff2TR, 0, SIZE_BUFF);
  snprintf(cpBuff2TR, SIZE_BUFF, "%s", buff2TR);
  
  // Get the answer from OROServer (or NULL if problem)
  if(buff2TR != NULL)
  {
    retVal = orosender(buff2TR);
  }
  
  if(gOtherClarify > 0)
  {
    // Answer result
    send(gSocketInfo.clientfd, buff2TR, strlen(buff2TR), 0);
    
    gOtherClarify -= 1;
    
    // Buffer should be cleaned if it is not used anymore
    free(buff2TR);
  }
  else if(retVal != 0)// Send back the received data thanks to the buff2TR
  {
    oroClientSendBack(buff2TR);

    // Buffer should be cleaned if it is not used anymore
    free(buff2TR);
  }
  else if(gFindRequest)
  {
    gFindRequest = false;
    retVal = request(cpBuff2TR);
  }
  else // Send back NULL
  {
    char nullStr[] = "[]";
    oroClientSendBack(nullStr);

    // Buffer should be cleaned if it is not used anymore
    free(buff2TR);
  }
}


/**
  * \fn void oroClientSendBack(char * buff2Send)
  * \param[in] buff2Send Answer from oro server to send back to the initial node 
  *
  * \brief This function just send back the answer from OroServer on a specific topic "oroChatterSendBack"
  */
void oroClientSendBack(char * buff2Send)
{
#warning "Author: -oroClientCallback: Returned Buffer from oroserver is not checked back"
  ros::NodeHandle n;
  
  // Message to send back
  std_msgs::String msg2Send;
  
  // Create the publisher
  ros::Publisher oroChatter_pub = n.advertise<std_msgs::String>("oroChatterSendBack", 1000);
  usleep(300000); // Necessary to initialize the ros system

  ros::Rate loop_rate(10);
  
  ROS_INFO("- Send back: --%s--",buff2Send);
  

  // Reconstruct the mesasge to send
  std::string s(buff2Send);
  msg2Send.data = s;
  
  // Publish the returned message
  oroChatter_pub.publish(msg2Send);
  
  usleep(500000);
  
}

int request(char * request)
{
  int sock;
  struct sockaddr_in server;
  char server_reply[SIZE_BUFF];
  
  char * subStr = NULL;
   
  //Create socket
  sock = socket(AF_INET , SOCK_STREAM , 0);
  if (sock == -1)
  {
      printf("Could not create socket");
      return -1;
  }
   
  server.sin_addr.s_addr = inet_addr(OTHER_ADDRESS);
  server.sin_family = AF_INET;
  server.sin_port = htons( OTHER_PORT );

  //Connect to remote server
  if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0)
  {
      ROS_INFO("- Error: Request connect failed");
      return -1;
  }
  
  ROS_INFO("- Request message: --%s--", request);
   
  //Send some data
  if( send(sock , request , strlen(request) , 0) < 0)
  {
      ROS_INFO("- Error: Request send failed");
      return -1;
  }
   
  //Receive a reply from the server
  if( recv(sock , server_reply , SIZE_BUFF , 0) < 0)
  {
      ROS_INFO("- Error: Request recv failed");
      return -1;
  }
   
  ROS_INFO("- Request server reply: --%s--",server_reply);
  
  close(sock);
  return 0;
}
