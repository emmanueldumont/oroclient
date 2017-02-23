#include "ros/ros.h"
#include "std_msgs/String.h"

#include <orolib/orolib.hpp>

#include <sstream>

void oroAnswerCallback(const std_msgs::String::ConstPtr& msg)
{
  char * str = NULL; // Local copy of the message received from the connected chatter
  char * knowledge = NULL; // Knowledge asked by the client
  
  char * strStart = NULL;
  char * strStop = NULL;
  char subStr[128];
  
  int sizeStr = 0;
    
  ROS_INFO ("- Parsing received message");
  
  // Get size of the received buffer 'including the '\0'
  sizeStr = strlen(msg->data.c_str())+1;
  
  // Allocate and add the data from the received packet to the working buffer
  str = (char *) malloc( sizeStr * sizeof(char));
  snprintf(str, sizeStr,"%s",msg->data.c_str());
  
  // Reinit data
  sizeStr = 0;
  
  // Parse with the Delimiter and get the knowledge if any
  
  ROS_INFO("%s", msg->data.c_str());
  
  memset(subStr, 0, 128);
    
  // Preprocessing to remove OK and #end#
  strStart = strchr(str, '\"');
  
  // If there is knowledge
  if(strStart != NULL)
  {
    // Find the last occurence of this delimiter
    strStop = strrchr(str, '\"');
    int size = strStop-strStart+1;
    
    // Create the substring
    memcpy(subStr, strStart, (size<128)?size:128);
    
    // Extract individual knowlege
    knowledge = strtok (subStr,"\",");
    do
    {
      ROS_INFO("found %s\n", knowledge);
      
      char cmdMove[256];
      snprintf(cmdMove, 256, "python /home/oculus/workspace/script/reachWP.py %s",knowledge);
      
      knowledge = strtok (NULL, "\",");
    }while (knowledge !=NULL);
  }
  else // If no Knowledge has been returned
  {
    ROS_INFO("- Warning: No information found");
  }

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "dummyTalker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher oroChatter_pub = n.advertise<std_msgs::String>("oroChatter", 1000);
  usleep(300000); // Necessary to initialize the ros system

  // Create the subscriber
	ros::NodeHandle ns;
	ros::Subscriber sub = ns.subscribe("oroChatterSendBack", 1000, oroAnswerCallback);
	
  ros::Rate loop_rate(10);

  /**
   * This is a message object. You stuff it with data, and then publish it.
  */
  std_msgs::String msg;

  std::stringstream ss;
  
  char enumCmd = 0;
  enumCmd = (char)CMD_ADD_PROP;
  ss << "BigBrother#"<<enumCmd<<"#remi#isIn#Kitchen,remi rdf:type Human";
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());
  
  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  //oroChatter_pub.publish(msg);
  
  //sleep(1);
  

  
  ss.str("");
  enumCmd = (char)CMD_FIND;
  ss << "BigBrother#"<<enumCmd<<"remi#isIn#?o";
  msg.data = ss.str();
  
  ROS_INFO("%s", msg.data.c_str());
  
  oroChatter_pub.publish(msg);
  
  // ros::spin() will enter a loop, pumping callbacks.
  ros::spin();

  return 0;
}
