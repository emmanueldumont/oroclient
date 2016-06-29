/**
  * \file msgParser.cpp
  * \brief This file contains two main function. One which parses a received command from a client,
  * another one which sends a command, asked by a client, to OROServer
  * \author DUMONT Emmanuel
  * \date 02/2016
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>

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

#include <orolib/orolib.hpp>
#include "msgParser.h"
#include "mngCommand.h"

/**
  * \fn char * parserMsgFromClient(const std_msgs::String::ConstPtr& msg)
  * \param[in] msg ROS message received
  *
  * \return Message to send to OROServer if any, NULL otherwise
  * 
  * \brief This function analyses the received message from ROS (ID, command, etc.).
  *        After analysis, it calls the corresponding command's function (if any) which will construct and send back an answer to send to OROServer.
  *        This answer is returned or NULL if the message was not accurate
  */
char * parserMsgFromClient(const std_msgs::String::ConstPtr& msg)
{
  char * retMsg2Send = NULL; // Returned message which will be send to oro server

  char * id = NULL; // ID of the sender
  char * cmd = NULL; // Command asked by the client
  char * str = NULL; // Local copy of the message received from the connected chatter
  int sizeStr = 0;
    
  ROS_INFO ("- Parsing received message");
  
  // Get size of the received buffer 'including the '\0'
  sizeStr = strlen(msg->data.c_str())+1;
  
  // Allocate and add the data from the received packet to the working buffer
  str = (char *) malloc( sizeStr * sizeof(char));
  snprintf(str, sizeStr,"%s",msg->data.c_str());
  
  // Reinit data
  sizeStr = 0;
  
  // Parse with the Delimiter and take the command
  // The id received from the client is not used currently
  id = strtok (str,DELIMITER);
  
  if(id != NULL)
  {
    // Work on the asked command
    cmd = strtok (NULL,DELIMITER);
    
    if(cmd != NULL)
    {
      switch(cmd[0])
      {
        // Command Add Instance
        case CMD_ADD_INST : retMsg2Send = addInstanceClassManager(str);
        break;
        // Command Add Instance
        case CMD_ADD_PROP : retMsg2Send = addInstanceManager(str);
        break;
        // Command Find something
        case CMD_FIND : retMsg2Send = findCommandManager(str);
        break;
        // Command Remove
        case CMD_REMOVE : retMsg2Send = rmvCommandManager(str);
        break;
        
        case CMD_CLEAR : retMsg2Send = clearInstanceManager(str);
        break;
        
        // Unknown command
        default :
        {
          ROS_INFO("\t- Error parsing command: Unknown command");
          free(retMsg2Send);
          free(str);
          return NULL;
        }
      }
    }
    else
    {
      ROS_INFO("\t- Error: No command found");
      free(str);
      return NULL;
    }
  }
  else
  {
    ROS_INFO("\t- Error: No command found");
    free(str);
    return NULL;
  }
  
  return retMsg2Send;
}


/**
  * \fn int orosender(char * buff2TR)
  * \param[in, out] buff2TR Message to send to OROServer. Answer is written in this buffer
  *
  * \return 0 If problems occured, 1 otherwise
  * 
  * \brief This function is call by several other function. If a problem occurs, the input string is freed and 0 is return.
  *        Otherwise 1 is return and the answer of OROServer is written in the input/output buffer : buff2TR
  */
int orosender(char * buff2TR)
{
  int sinSize = 0;            // Size of the sin structure
  int ret = 0;                // Return value
  SOCKET sock;                // Socket to open
  SOCKADDR_IN sin = { 0 };    // Stock information
  struct timeval tv;          // Select Timeout
  int  rcvSize = 0;           // Size of the received message
      
  // Update data
  sin.sin_addr     =  gSocketInfo.sin.sin_addr;
  sin.sin_port     =  gSocketInfo.sin.sin_port;
  sin.sin_family   =  gSocketInfo.sin.sin_family;
  sinSize          =  gSocketInfo.sinSize;
  sock             =  gSocketInfo.socket;
  
    
  if(buff2TR != NULL)
  {

    ROS_INFO("\t- Buffer is '%s' size is %d",buff2TR, (int) strlen(buff2TR));
    if( (ret = sendto(sock, (char *) buff2TR, strlen(buff2TR), 0, (SOCKADDR *) &sin, sinSize)) < 0)
    {
        ROS_INFO ("\t- Error on TX command");
        free(buff2TR);
        buff2TR = NULL;
        
        return 0;
    }

    // Get the answer from the server. Timeout 5 secs
    ROS_INFO("- Get the answer within 5sec");

    // Clear the set before use it
    FD_ZERO(&gSocketInfo.rdfs);

    // add the socket to the set
    FD_SET(sock, &gSocketInfo.rdfs);

    // Timeout managementcd
    tv.tv_sec  = TIMEOUT_SVR;
    tv.tv_usec = 0;

    // If problem
    if( (ret = select(sock + 1, &gSocketInfo.rdfs, NULL, NULL, &tv)) < 0)
    {
        ROS_INFO("\t- Error: when receiving packet ( select() )");
        free(buff2TR);
        buff2TR = NULL;
        return 0;
    }
    // Else if timeout
    else if(!ret)
    {
        ROS_INFO("\t- Error: timeout read from server");
        free(buff2TR);
        buff2TR = NULL;
        return 0;
    }
    // If everything is fine: receive the packet
    else if(FD_ISSET(sock, &gSocketInfo.rdfs))
    {
        // Clear the input buffer and set sinSize
        memset(buff2TR,'\0',SIZE_BUFF);
        sinSize = sizeof(sin);
        
        ROS_INFO("- Get data2");
        
        if( (rcvSize = recvfrom(sock, (char*) buff2TR, SIZE_BUFF-1, 0, (SOCKADDR *) &sin, (socklen_t*) &sinSize)) <1)
        {
            ROS_INFO("\t- recv() error");
            free(buff2TR);
            buff2TR = NULL;
            return 0;
        }

        ROS_INFO("\t- The returned paquet is : '%s'",buff2TR);
        return 1;
    }
  }
  else
  {
    ROS_INFO("\t- Packet not send");
    return 0;
  }
}
