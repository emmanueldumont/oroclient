/**
  * \file utils.h
  * \brief This file contains declaration of functions used as a toolbox
  * \author DUMONT Emmanuel
  * \date 02/2016
  */

#ifndef UTILS_H
#define UTILS_H

/**
  * \fn char* str_replace(char *orig,const char* rep,const char* with)
  * \param[in] orig Initial string with parts to replace
  * \param[in] rep Parts which has to be replaced
  * \param[in] with Parts which has to be replaced with
  *
  * \return Final string with replacement made or NULL if error occured
  * 
  * \brief This function looks in the "orig" string occurence of "rep" and change them by "with"
  * You must free the result if result is non-NULL
  *
  * \Author Credits to jmucchiello - Stackoverflow.com
  */
char* str_replace(char *orig,const char * rep,const char * with);


#endif // CLIENT_H_INCLUDED
