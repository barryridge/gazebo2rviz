#!/usr/bin/env python

from __future__ import print_function

import rospy
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
import tf_conversions.posemath as pm
from tf.transformations import *

import pysdf

tfBroadcaster = None
submodelsToBeIgnored = []
lastUpdateTime = None
updatePeriod = 0.05
model_cache = {}


def is_ignored(link_name):
  for ignored_submodel in submodelsToBeIgnored:
    if link_name.startswith(ignored_submodel + '::'):
      return True
  return False


def on_link_states_msg(link_states_msg):
  """
  Publish tf for each model in current Gazebo world
  """
  global lastUpdateTime
  sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
  if sinceLastUpdateDuration.to_sec() < updatePeriod:
    return
  lastUpdateTime = rospy.get_rostime()

  poses = {'gazebo_world': identity_matrix()}
  for (link_idx, link_name) in enumerate(link_states_msg.name):
    poses[link_name] = pysdf.pose_msg2homogeneous(link_states_msg.pose[link_idx])
    #print('%s:\n%s' % (link_name, poses[link_name]))

  for (link_idx, link_name) in enumerate(link_states_msg.name):
    modelinstance_name = link_name.split('::')[0]
    model_name = pysdf.name2modelname(modelinstance_name)
    if model_name in submodelsToBeIgnored:
      continue
    if not model_name in model_cache:
      rospy.loginfo("gazebo2tf_node: Loading sdf model: {}".format(model_name))
      sdf = pysdf.SDF(model=model_name, ignore_submodels=submodelsToBeIgnored)
      if sdf:
        rospy.loginfo("gazebo2tf_node: Sucessfully loaded sdf model: {}".format(model_name))
      else:
        rospy.loginfo("gazebo2tf_node: Failed to load sdf model: {}".format(model_name))
      model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
      if model_cache[model_name]:
        rospy.loginfo('gazebo2tf_node: Loaded model: %s' % model_cache[model_name].name)
      else:
        rospy.loginfo('gazebo2tf_node: Unable to load model: %s' % model_name)
    model = model_cache[model_name]
    link_name_in_model = link_name.replace(modelinstance_name + '::', '')
    if model:
      try:
        link = model.get_link(link_name_in_model)
      except Exception as e:
        rospy.logwarn("gazebo2tf_node: Unable to find link {} in model {}: {}".format(link_name_in_model, model_name, repr(e)))
        continue
      try:
        parent_link = link.tree_parent_joint.tree_parent_link
        parent_link_name = parent_link.get_full_name()
        #print('parent:', parent_link_name)
        parentinstance_link_name = parent_link_name.replace(model_name, modelinstance_name, 1)
      except Exception as e:
        # direct child of world
        parentinstance_link_name = 'gazebo_world'
    else: # Not an SDF model
        parentinstance_link_name = 'gazebo_world'
    #print('parentinstance:', parentinstance_link_name)
    if is_ignored(parentinstance_link_name):
      rospy.loginfo("Ignoring TF %s -> %s" % (parentinstance_link_name, link_name))
      continue
    pose = poses[link_name]
    parent_pose = poses[parentinstance_link_name]
    rel_tf = concatenate_matrices(inverse_matrix(parent_pose), pose)
    translation, quaternion = pysdf.homogeneous2translation_quaternion(rel_tf)
    #print('Publishing TF %s -> %s: t=%s q=%s' % (pysdf.sdf2tfname(parentinstance_link_name), pysdf.sdf2tfname(link_name), translation, quaternion))
    tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), pysdf.sdf2tfname(link_name), pysdf.sdf2tfname(parentinstance_link_name))



def main():
  rospy.init_node('gazebo2tf')

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels', '').split(';')
  rospy.loginfo('gazebo2tf_node: Ignoring submodels of: ' + str(submodelsToBeIgnored))

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_states_msg)

  rospy.loginfo('Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
