/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "imu_visual.h"

#include "smacc_rviz_display.h"

namespace smacc_rviz_plugin
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
SmaccRvizDisplay::SmaccRvizDisplay()
{

  /*
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
                                                    */

  current_state_ = new rviz::StringProperty( "CurrentState", "",
                                                    "current smacc state",
                                                    this, SLOT( updateCurrentState() ));

  
  topic_property_ = new rviz::RosTopicProperty( "Topic", "",
                                            "", "",
                                            this, SLOT( updateTopic() ));

  QString message_type = QString::fromStdString( ros::message_traits::datatype<smacc_msgs::SmaccStatus>() );
      topic_property_->setMessageType( message_type );
      topic_property_->setDescription( message_type + " topic to subscribe to." );


  //history_length_property_->setMin( 1 );
  //history_length_property_->setMax( 100000 );
}

  void SmaccRvizDisplay::updateTopic()
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  void SmaccRvizDisplay::subscribe()
  {
      current_state_->setValue(QString(""));
      sub_ = nh_.subscribe<smacc_msgs::SmaccStatus>(topic_property_->getTopic().toStdString(), 1, &SmaccRvizDisplay::processMessage , this);
  }

  void SmaccRvizDisplay::unsubscribe()
  {
    current_state_->setValue(QString(""));
    sub_.shutdown();

  }
// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void SmaccRvizDisplay::onInitialize()
{
  //MFDClass::onInitialize();
  current_state_->setValue(QString(""));
}

SmaccRvizDisplay::~SmaccRvizDisplay()
{
}

// Clear the visuals by deleting their objects.
void SmaccRvizDisplay::reset()
{
  //MFDClass::reset();
  //visuals_.clear();
  current_state_->setValue(QString(""));
}

void SmaccRvizDisplay::updateCurrentState()
{  
}

// This is our callback to handle an incoming message.
void SmaccRvizDisplay::processMessage( const smacc_msgs::SmaccStatus::ConstPtr& msg )
{
  std::string concatenated;

  for(auto& state: msg->current_states)
  {
    concatenated = concatenated + std::string("/")+ state;
  }

  ROS_DEBUG("current state: %s", concatenated.c_str());
  current_state_->setValue(QString(concatenated.c_str()));
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  /*
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  boost::shared_ptr<ImuVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ImuVisual( context_->getSceneManager(), scene_node_ ));
  }*/

  // Now set or update the contents of the chosen visual.
  //visual->setMessage( msg );
  //visual->setFramePosition( position );
  //visual->setFrameOrientation( orientation );

  /*
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor( color.r, color.g, color.b, alpha );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);
  */
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(smacc_rviz_plugin::SmaccRvizDisplay,rviz::Display )
// END_TUTORIAL