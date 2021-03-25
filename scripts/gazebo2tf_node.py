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
submodelsToBeIncluded = []
submodelsToBeIgnored = []
lastUpdateTime = None
updatePeriod = 0.05
model_cache = {}

def is_included(link_name):
  if link_name == 'gazebo_world':
    return True
  if not submodelsToBeIncluded:
    return True
  for included_submodel in submodelsToBeIncluded:
    if link_name.startswith(included_submodel + '::'):
      return True
  return False

def is_ignored(link_name):
  for ignored_submodel in submodelsToBeIgnored:
    if link_name.startswith(ignored_submodel + '::'):
      return True
  return False

def load_model(model_name):
  if not model_name in model_cache:
    model_cache[model_name] = None
    sdf = pysdf.SDF(model=model_name, ignore_submodels=submodelsToBeIgnored)
    model_cache[model_name] = sdf.world.models[0] if len(sdf.world.models) >= 1 else None
  model = model_cache[model_name]

  return model

def get_parentinstance_link_name(model, model_name, modelinstance_name, link_name):
    link_name_in_model = link_name.replace(modelinstance_name + '::', '')
    if model:
      try:
        link = model.get_link(link_name_in_model)
      except Exception as e:
        rospy.logwarn('gazebo2tf_node: Unable to find link {} in model {}: {}'.format(link_name_in_model, model_name, repr(e)))
        return None
      try:
        parent_link = link.tree_parent_joint.tree_parent_link
        parent_link_name = parent_link.get_full_name()
        parentinstance_link_name = parent_link_name.replace(model_name, modelinstance_name, 1)
      except Exception:
        # direct child of world
        parentinstance_link_name = 'gazebo_world'
    else: # Not an SDF model
        parentinstance_link_name = 'gazebo_world'

    return parentinstance_link_name

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

  for (link_idx, link_name) in enumerate(link_states_msg.name):
    modelinstance_name = link_name.split('::')[0]

    try:
      model_name = modelinstance_name
      if (submodelsToBeIncluded and model_name not in submodelsToBeIncluded) or model_name in submodelsToBeIgnored:
        rospy.logdebug("gazebo2tf_node: Model instance {} not included".format(model_name))
        continue
      else:
        rospy.logdebug("gazebo2tf_node: Model instance {} included".format(model_name))
      model = load_model(model_name)
      parentinstance_link_name = get_parentinstance_link_name(model, model_name, modelinstance_name, link_name)
      assert(parentinstance_link_name)
    except Exception:
      try:
        model_name = pysdf.name2modelname(modelinstance_name)
        if (submodelsToBeIncluded and model_name not in submodelsToBeIncluded) or model_name in submodelsToBeIgnored:
          rospy.logdebug("gazebo2tf_node: Model {} not included".format(model_name))
          continue
        else:
          rospy.logdebug("gazebo2tf_node: Model {} included".format(model_name))
        model = load_model(model_name)
        parentinstance_link_name = get_parentinstance_link_name(model, model_name, modelinstance_name, link_name)
        assert(parentinstance_link_name) 
      except Exception as e:
        rospy.logdebug('gazebo2tf_node: Failed to find parent instance link name for '
                       'model {}, model instance {}: {}'.format(model_name, modelinstance_name, repr(e)))
        continue

    if not is_included(parentinstance_link_name) or is_ignored(parentinstance_link_name):
      rospy.loginfo('Ignoring TF {} -> {}'.format(parentinstance_link_name, link_name))
      continue
    pose = poses[link_name]
    parent_pose = poses[parentinstance_link_name]
    rel_tf = concatenate_matrices(inverse_matrix(parent_pose), pose)
    translation, quaternion = pysdf.homogeneous2translation_quaternion(rel_tf)
    tfBroadcaster.sendTransform(translation, quaternion, rospy.get_rostime(), pysdf.sdf2tfname(link_name), pysdf.sdf2tfname(parentinstance_link_name))


def main():
  rospy.init_node('gazebo2tf')

  global submodelsToBeIncluded
  submodelsToBeIncluded = rospy.get_param('~include_submodels', '').split(';')
  if submodelsToBeIncluded == ['']:
    submodelsToBeIncluded = []
  else:
    rospy.loginfo('gazebo2tf_node: Including submodels: ' + str(submodelsToBeIncluded))

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels', '').split(';')
  if submodelsToBeIgnored == ['']:
    submodelsToBeIgnored = []
  else:
    rospy.loginfo('gazebo2tf_node: Ignoring submodels: ' + str(submodelsToBeIgnored))

  global tfBroadcaster
  tfBroadcaster = tf.TransformBroadcaster()

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, on_link_states_msg)

  rospy.loginfo('gazebo2tf_node: Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
