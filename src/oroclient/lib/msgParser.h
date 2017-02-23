/**
  * \file msgParser.h
  * \brief This file contains Structure definition , Global Structures, divers utils things to manage the socket with OROServer (address, port,...)
  * \author DUMONT Emmanuel
  * \date 02/2016
  */

#ifndef CLIENT_H
#define CLIENT_H

#include <netdb.h> // Used for the typedef below


/*
 * \def INVALID_SOCKET -1 : Obvious
 */
#define INVALID_SOCKET -1
/*
 * \def SOCKET_ERROR -1 : Obvious
 */
#define SOCKET_ERROR -1

/*
 * \def closesocket(s) close(s) : Redefine a function name to be more explicit
 */
#define closesocket(s) close(s)

/*
 * \def ORO_ADDRESS "127.0.0.1" : IP Address of the OROServer (put as a String)
 */
#define     ORO_ADDRESS "127.0.0.1"
/*
 * \def ORO_PORT 6969 : PORT of the OROServer (put as a String)
 */
#define     ORO_PORT    6969

/*
 * \def ORO_ADDRESS "127.0.0.1" : IP Address of the OROServer (put as a String)
 */
#define     OTHER_ADDRESS "192.168.0.133"
/*
 * \def ORO_PORT 6969 : PORT of the OROServer (put as a String)
 */
#define     OTHER_PORT    32768

/*
 * \def TIMEOUT_SVR 5 : Define a server timeout identical for everybody
 */
#define     TIMEOUT_SVR 5

/*
 * \def SIZE_BUFF 256 : Buffer size identical for everybody
 */
#define     SIZE_BUFF 256

/*
 * \def gFindRequest : Global variable to indicate if the current request is a Find request
 */
bool gFindRequest;


/*
 * \struct  enCommand : Enum command -> Enum for the different implemented commands
 * \struct sockaddr_in SOCKADDR_IN : Redifined to avoid conflicts with constant definition (maj case letters)
 * \struct sockaddr SOCKADDR : Redifined to avoid conflicts with constant definition (maj case letters)
 * \struct in_addr IN_ADDR : Redifined to avoid conflicts with constant definition (maj case letters)
 */
typedef int SOCKET;
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr SOCKADDR;
typedef struct in_addr IN_ADDR;

/*
 * \struct  oroClGlobalSocket
 * \brief Definition of a global structure of information about the OROServer socket in order to use after the ROS Callback.
 *  Contains:
 *  SOCKET socket;
 *  SOCKADDR_IN sin;
 *  int sinSize;
 *  fd_set rdfs;
 */
typedef struct oroClGlobalSocket
{
    SOCKET socket;
    SOCKADDR_IN sin;
    int sinSize;
    fd_set rdfs;                        // Read descriptor
    SOCKET otherSocket;
    int clientfd;
}t_oroClGlobalSocket;

/*
 * \var struct oroClGlobalSocket gSocketInfo;
 * \brief Global Variable used in the context of accessing data in a ROS callback
 */
struct oroClGlobalSocket gSocketInfo;

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
char * parserMsgFromClient(const std_msgs::String::ConstPtr& msg);

/**
  * \fn int orosender(char * buff2TR)
  * \param[in, out] buff2TR Message to send to OROServer. Answer is written in this buffer
  *
  * \return 0 If problems occured, 1 otherwise
  * 
  * \brief This function is call by several other function. If a problem occurs, the input string is freed and 0 is return.
  *        Otherwise 1 is return and the answer of OROServer is written in the input/output buffer : buff2TR
  */
int orosender(char * buff2TR);

int request(char * request);


#endif // CLIENT_H_INCLUDED
