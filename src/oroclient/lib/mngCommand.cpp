/**
  * \file mngCommand.cpp
  * \brief This file contains functions used to manages the asked command from a client
  * \author DUMONT Emmanuel
  * \date 02/2016
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


// ROS lib
#include "ros/ros.h"
#include "std_msgs/String.h"

// Inner lib

#include <orolib/orolib.hpp>
#include "mngCommand.h"
#include "msgParser.h"
#include "utils.h"

/**
  * \fn char * addInstanceClassManager(char * str)
  * \brief This function analyses the received message from ROS (number of argument).
  *        After analysis, it constructs a command which should be like:
  * Form of what we should have step by step:
  * 
  * add
  * [kinect rdf:type VideoSensor]
  * #end#
  * 
  * "add\n[kinect rdf:type VideoSensor]\n#end#\n"
  * \param[in] str String which contains the received command
  *
  * \return Message to send to OROServer if any, NULL otherwise or if an error occured
  * 
  */
char * addInstanceClassManager(char * str)
{
  int sizeStr = 0;   // Size of a string
  char * retMsg2Send = NULL; // Buffer to send to oroserver
  
  // Allocate the buffer to send to oroserver
  retMsg2Send = (char *) malloc( SIZE_BUFF * sizeof(char));
  memset(retMsg2Send, '\0', SIZE_BUFF);
  
  
  
  // As strtok has been initialized in the "parserMsgFromClient" function, we can just call it again

  // Declare the necessary number of char* for the argument
  char * argN[2] = {NULL, NULL};
  
  // Get arg1 &2
  argN[0] = strtok (NULL,DELIMITER);
  argN[1] = strtok (NULL,DELIMITER);
  
  if( (argN[0] == NULL) || (argN[1] == NULL) )
  {
    ROS_INFO("\t- ERROR when parsing command, Not enough Argument in the asked command. Should be ID#Command#Arg1#Arg2#");
    free(retMsg2Send);
    free(str);
    
    return NULL;
  }
  
  // Check if size of the buffer is enough :
  sizeStr = strlen("add\n[") + strlen(argN[0]) + strlen(" rdf:type ") + strlen(argN[1]) + strlen ("]\n#end#\n");
  if( sizeStr >= SIZE_BUFF)
  {
    ROS_INFO("\t- ERROR when parsing command, message is too long");
    
    free(retMsg2Send);
    free(str);
    
    
    return NULL;
  }
          
  // From now, we are sure that there is enough space
  // But as "Prudence as the "mother" of all virtues"
  sizeStr = snprintf(retMsg2Send, SIZE_BUFF,"add\n[%s rdf:type %s]\n#end#\n",argN[0], argN[1]);

  return retMsg2Send;
}

/**
  * \fn char * addInstanceManager(char * str)
  * \brief This function analyses the received message from ROS (number of argument).
  *        After analysis, it constructs a command which should be like:
  * Form of what we should have step by step:
  * 
  * add
  * [kinect canSee manu]
  * #end#
  * 
  * "add\n[kinect canSee manu]\n#end#\n"
  *
  * \param[in] str String which contains the received command
  *
  * \return Message to send to OROServer if any, NULL otherwise or if an error occured
  * 
  */
char * addInstanceManager(char * str)
{
  int sizeStr = 0;   // Size of a string
  char * retMsg2Send = NULL; // Buffer to send to oroserver
  
  // Allocate the buffer to send to oroserver
  retMsg2Send = (char *) malloc( SIZE_BUFF * sizeof(char));
  memset(retMsg2Send, '\0', SIZE_BUFF);

  // As strtok has been initialized in the "parserMsgFromClient" function, we can just call it again
  
  // Declare the necessary number of char* for the argument
  char * argN[3] = {NULL, NULL, NULL};
  
  // Get arg1 &2 &3
  argN[0] = strtok (NULL,DELIMITER);
  argN[1] = strtok (NULL,DELIMITER);
  argN[2] = strtok (NULL,DELIMITER);
  
  if( (argN[0] == NULL) || (argN[1] == NULL) || (argN[2] == NULL) )
  {
    ROS_INFO("\t- ERROR when parsing command, Not enough Argument in the asked command. Should be ID#Command#Arg1#Arg2#Arg3");
    
    free(retMsg2Send);
    free(str);
    
    
    return NULL;
  }
  
  // Check if size of the buffer is enough :
  sizeStr = strlen("add\n[") + strlen(argN[0]) + strlen(argN[1]) + strlen(argN[2]) + strlen ("  ]\n#end#\n"); // We add at the end 2 spaces: there is one between arg1 and Arg2 & Arg2 and arg3
  if( sizeStr >= SIZE_BUFF)
  {
    ROS_INFO("\t- ERROR when parsing command, message is too long");
    
    free(retMsg2Send);
    free(str);
    
    return NULL;
  }
          
  // From now, we are sure that there is enough space
  // But as "Prudence as the "mother" of all virtues"
  sizeStr = snprintf(retMsg2Send, SIZE_BUFF,"add\n[%s %s %s]\n#end#\n",argN[0], argN[1], argN[2]);
  
  return retMsg2Send;
  
}
      

/**
  * \fn char * addInstanceManager(char * str)
  * \brief This function analyses the received message from ROS (number of argument).
  *        After analysis, it constructs a command which should be like:
  * Form of what we should have step by step:
  * 
  * add
  * [kinect canSee manu]
  * #end#
  * 
  * "add\n[kinect canSee manu]\n#end#\n"
  *
  * \param[in] str String which contains the received command
  *
  * \return Message to send to OROServer if any, NULL otherwise or if an error occured
  * 
  */
char * clearInstanceManager(char * str)
{
  int sizeStr = 0;   // Size of a string
  char * retMsg2Send = NULL; // Buffer to send to oroserver
  
  // Allocate the buffer to send to oroserver
  retMsg2Send = (char *) malloc( SIZE_BUFF * sizeof(char));
  memset(retMsg2Send, '\0', SIZE_BUFF);

  // As strtok has been initialized in the "parserMsgFromClient" function, we can just call it again
  
  // Declare the necessary number of char* for the argument
  char * argN[3] = {NULL, NULL, NULL};
  
  // Get arg1 &2 &3
  argN[0] = strtok (NULL,DELIMITER);
  argN[1] = strtok (NULL,DELIMITER);
  argN[2] = strtok (NULL,DELIMITER);
  
  if( (argN[0] == NULL) || (argN[1] == NULL) || (argN[2] == NULL) )
  {
    ROS_INFO("\t- ERROR when parsing command, Not enough Argument in the asked command. Should be ID#Command#Arg1#Arg2#Arg3");
    
    free(retMsg2Send);
    free(str);
    
    
    return NULL;
  }
  
  // Check if size of the buffer is enough :
  sizeStr = strlen("clear\n[") + strlen(argN[0]) + strlen(argN[1]) + strlen(argN[2]) + strlen ("  ]\n#end#\n"); // We add at the end 2 spaces: there is one between arg1 and Arg2 & Arg2 and arg3
  if( sizeStr >= SIZE_BUFF)
  {
    ROS_INFO("\t- ERROR when parsing command, message is too long");
    
    free(retMsg2Send);
    free(str);
    
    return NULL;
  }
          
  // From now, we are sure that there is enough space
  // But as "Prudence as the "mother" of all virtues"
  sizeStr = snprintf(retMsg2Send, SIZE_BUFF,"clear\n[%s %s %s]\n#end#\n",argN[0], argN[1], argN[2]);
  
  return retMsg2Send;
  
}


/**
  * \fn char * findCommandManager(char * str)
  * \brief NOT IMPLEMENTED YET
  * --
  *
  * This function analyses the received message from ROS (number of argument).
  *        After analysis, it constructs a command which should be like:
  * Form of what we should have step by step:
  * 
  * find
  * o
  * [?o canSee manu]
  * #end#
  * 
  * "find\no\n[?o canSee manu]\n#end#\n"
  * 
  * \param[in] str String which contains the received command
  *
  * \return Message to send to OROServer if any, NULL otherwise or if an error occured
  * 
  */
char * findCommandManager(char * str)
{
  #warning "Author: -findCommandManager: Not implemented yet"
  int sizeStr = 0;   // Size of a string
  char * retMsg2Send = NULL; // Buffer to send to oroserver
  
  // Allocate the buffer to send to oroserver
  retMsg2Send = (char *) malloc( SIZE_BUFF * sizeof(char));
  memset(retMsg2Send, '\0', SIZE_BUFF);
    
  // As strtok has been initialized in the "parserMsgFromClient" function, we can just call it again
  // Declare the necessary number of char* for the argument
  char * argN[2] = {NULL, NULL};
  
  // Get arg1 &2
  argN[0] = strtok (NULL,DELIMITER);
  argN[1] = strtok (NULL,DELIMITER);
  
  if( (argN[0] == NULL) || (argN[1] == NULL) )
  {
    ROS_INFO("\t- ERROR when parsing command, message is too long");
    
    
    free(retMsg2Send);
    free(str);
    return NULL;
  }
  
  // Check if size of the buffer is enough :
  sizeStr = strlen("find\no\n[?o ") + strlen(argN[0]) + strlen(argN[1]) + strlen (" ]\n#end#\n"); // We add at the end 1 space: there is one between arg1 and Arg2
  if( sizeStr >= SIZE_BUFF)
  {
    ROS_INFO("\t- ERROR when parsing command, Not enough ");
    
    free(retMsg2Send);
    free(str);
    
    return NULL;
  }
          
  // From now, we are sure that there is enough space
  // But as "Prudence as the "mother" of all virtues"
  sizeStr = snprintf(retMsg2Send, SIZE_BUFF,"find\no\n[?o %s %s]\n#end#\n",argN[0], argN[1]);
  
  return retMsg2Send;
}


/**
  * \fn char * rmvCommandManager(char * str)
  * \brief This function analyses the received message from ROS (number of argument).
  *        After analysis, it constructs a first command which ask to OROServer to send information about the only argument received in the string
  *        Then if OROServer send back information and after some processing of the answer, it asks to remove everything should be like:
  *        Form of what we should have step by step:
  *        
  *        getInfos
  *        remi
  *        #end#
  *        
  *        "getInfos\nremi\n#end#\n"
  *       
  *        from oro :
  *       
  *        ok
  *        ["remi isAlsoIn Kitchen","remi type Thing","remi isIn Kitchen","remi differentFrom myself","remi sameAs remi","remi type Human","remi type Actor","remi differentFrom Kitchen"]
  *        #end#
  *       
  *        Need to replace "type" with "rdf:type" and remove the "ok" and "#end#" in order to remove everything :
  *       
  *       remove
  *       ["remi isAlsoIn Kitchen","remi rdf:type Thing","remi isIn Kitchen","remi differentFrom myself","remi sameAs remi","remi rdf:type Human","remi rdf:type Actor","remi differentFrom Kitchen"]
  *       #end"
  *       
  *       "remove\n["remi isAlsoIn Kitchen","remi rdf:type Thing","remi isIn Kitchen","remi differentFrom myself","remi sameAs remi","remi rdf:type Human","remi rdf:type Actor","remi differentFrom Kitchen"]\n#end#\n"
  *   
  * \param[in] str String which contains the received command
  *
  * \return Message to send to OROServer if any, NULL otherwise or if an error occured
  *     
  */
char * rmvCommandManager(char * str)
{  
  int valRet = 0; // Return value of called functions
  int sizeStr = 0;  // Size of a returned string
  char * argN = NULL; // argN argument passed to oroclient to remove from the ontology
  char * strRmv = NULL; // String used to removed every statement of the information this specific string ; must not be allocate (done in str_replacement)
  char * finStrRmv = NULL; // Final string used to removed every statement of the information this specific string
  char * retMsg2Send = NULL;    // Buffer to send to oroserver
  
  const char* cType = "type";
  const char* cOkn = "ok\n";
  const char* cEnd = "\n#end#\n";
  const char* cRdf = "rdf:type";
  
  // Buffer allocation
  retMsg2Send = (char *) malloc( SIZE_BUFF * sizeof(char));
  
  memset(retMsg2Send, '\0', SIZE_BUFF);
  
  // As strtok has been initialized in the "parserMsgFromClient" function, we can just call it again
  
  // Get arg1 &2
  argN = strtok (NULL,DELIMITER);
  
  
  if(argN == NULL)
  {
    ROS_INFO("\t- ERROR when parsing command, Not enough Argument in the asked command. Should be ID#Command#Arg1#Arg2");
    
    free(retMsg2Send);
    free(str);
    return NULL;
  }
  
  char * tmp = (char *) malloc( (strlen(argN)+1) *sizeof(char)); // Temps is used here cause argN depends on 'str' and lot of string will depend on argN thus cannot be free or else
  strcpy(tmp, argN);
  
  ///////////////////
  //               //
  //    getInfos   //
  //               //
  ///////////////////
  
  // Check if size of the buffer is enough :
  sizeStr = strlen("getInfos\n\n#end#\n") + strlen(argN);
  if( sizeStr >= SIZE_BUFF)
  {
    ROS_INFO("\t- ERROR when parsing command, message is too long");
    
    free(retMsg2Send);
    free(str);
    
    return NULL;
  }
  
  // From now, we are sure that there is enough space
  // But as "Prudence as the "mother" of all virtues"
  sizeStr = snprintf(retMsg2Send, SIZE_BUFF,"getInfos\n%s\n#end#\n",tmp);
  
  ROS_INFO("- Send to oroServer: '%s'",retMsg2Send);
  
  // Send the getInfos to Oroserver
  valRet = orosender(retMsg2Send);
  
  // If no error
  if( (valRet != 0) && (retMsg2Send != NULL) )
  {
    // Process the received string to be able to remove everything - retMsg2Send will be free()
    //    |- Consists of removing the #end# add the end of the response
    //    |- Consists of deleting the "ok\n" string at the beginning of the answer
    //    |_ Consists of replacing all 'type' with 'rdf:type'
    char * strRmv1 = str_replace(retMsg2Send, cType, cRdf);
    char * strRmv2 = str_replace(strRmv1, cEnd, NULL);
    strRmv = str_replace(strRmv2, cOkn, NULL);
    
    free(strRmv1);
    free(strRmv2);

    // If nothing is returned, quit
    if(strRmv == NULL)
    {
      ROS_INFO("\t- ERROR when processing answer from OroServer");
      
      free(str);
      
      return NULL;
    }
    
    // create the remove packet :
    sizeStr = strlen(strRmv) + 15; // 15 = strlen("remove\n\n#end#\n") +1; // 1 Corresponds to '\0'
    
    finStrRmv= (char *) malloc(SIZE_BUFF * sizeof(char));
    memset(finStrRmv, '\0', SIZE_BUFF);
    
    snprintf(finStrRmv, sizeStr, "remove\n%s\n#end#\n",strRmv);
    
    free(strRmv);
    free(str);
    free(tmp);
    
  }
  else
  {
    if(retMsg2Send != NULL)
    {
      free(retMsg2Send);
      ROS_INFO("\t- ERROR empty returned buffer");
    }
    free(str);
    
    ROS_INFO("\t- ERROR no answer from the server");
    return NULL;
  }
  
  return finStrRmv;
}
