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

// Inner lib
#include "lib/msgParser.h"



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

    // ros::spin() will enter a loop, pumping callbacks.
    ros::spin();

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
  
  ROS_INFO("- Received packet: --%s--", msg->data.c_str());
  
  // Parse the message and manage what to do
  buff2TR = parserMsgFromClient(msg);
  
  // Get the answer from OROServer (or NULL if problem)
  if(buff2TR != NULL)
  {
    retVal = orosender(buff2TR);
  }
  
  // Send back the received data thanks to the buff2TR
  if(retVal != 0)
  {
    oroClientSendBack(buff2TR);

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
  
  ros::spinOnce();
  loop_rate.sleep();
}
