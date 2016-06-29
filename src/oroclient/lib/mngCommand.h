/**
  * \file mngCommand.h
  * \brief This file contains declaration of functions used to manages the asked command from a client
  * \author DUMONT Emmanuel
  * \date 02/2016
  */

#ifndef MNGCOMMAND_H
#define MNGCOMMAND_H

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
char * addInstanceClassManager(char * str);
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
char * addInstanceManager(char * str);
char * clearInstanceManager(char * str);
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
char * findCommandManager(char * str);
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
char * rmvCommandManager(char * str);


#endif
