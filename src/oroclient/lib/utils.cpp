/**
  * \file utils.cpp
  * \brief This file contains declaration of functions used as a toolbox
  * \author DUMONT Emmanuel
  * \date 02/2016
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "utils.h"


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
char* str_replace(char *orig,const char* rep,const char* with)
{
    char *result; // the return string
    char *ins;    // the next insert point
    char *tmp;    // varies
    int len_rep;  // length of rep
    int len_with; // length of with
    int len_front; // distance between rep and end of last rep
    int count;    // number of replacements

    if (!orig)
        return NULL;
    if (!rep)
        rep = "";
    len_rep = strlen(rep);
    if (!with)
        with = "";
    len_with = strlen(with);

    ins = orig;
    for (count = 0; tmp = strstr(ins, rep); ++count) {
        ins = tmp + len_rep;
    }
    
    // first time through the loop, all the variable are set correctly
    // from here on,
    //    tmp points to the end of the result string
    //    ins points to the next occurrence of rep in orig
    //    orig points to the remainder of orig after "end of rep"
    tmp = result = (char *)malloc(strlen(orig) + ((len_with - len_rep) * count) + 1);

    if (!result)
        return NULL;
        

    while (count--) {
        ins = strstr(orig, rep);
        len_front = ins - orig;
        tmp = strncpy(tmp, orig, len_front) + len_front;
        tmp = strcpy(tmp, with) + len_with;
        orig += len_front + len_rep; // move to next "end of rep"
    }
    
    strcpy(tmp, orig);
        
    return result;
}
