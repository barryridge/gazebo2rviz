#!/usr/bin/env python

from __future__ import print_function

import os
import numpy as np
import argparse

import rospy
import rospkg
import tf
import tf2_ros
import tf2_geometry_msgs
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import trimesh

import pysdf
from gazebo2rviz import *


rospack = rospkg.RosPack()
tfBuffer = None
tfListener = None
updatePeriod = 0.5
use_collision = False
submodelsToBeIncluded = []
submodelsToBeIgnored = []
markerPub = None
world = None
model_cache = {}
lastUpdateTime = None
worldsdf = None
latch = False
max_messages = -1
message_count = 0
lifetime = 1
trimeshes = []
mesh_output_file = None


def publish_link_marker(link, full_linkname, **kwargs):
  global trimeshes
  full_linkinstancename = full_linkname
  if 'model_name' in kwargs and 'instance_name' in kwargs:
    full_linkinstancename = full_linkinstancename.replace(kwargs['model_name'], kwargs['instance_name'], 1)
  marker_msgs = link2marker_msg(link, full_linkinstancename, use_collision, lifetime)
  if len(marker_msgs) > 0:
    for marker_msg in marker_msgs:
      markerPub.publish(marker_msg)
      if mesh_output_file:
        marker_mesh = marker_msg_to_trimesh(marker_msg)
        if marker_mesh:
          trimeshes.append(marker_mesh)

def load_model(model_name):
  if not model_name in model_cache:
    model_cache[model_name] = None
    if worldsdf:
      for model in worldsdf.world.models:
        if model.name == model_name:
          model_cache[model_name] = model
          break
    else:
      sdf = pysdf.SDF(model=model_name, ignore_submodels=submodelsToBeIgnored)
      if len(sdf.world.models) >= 1:
        model_cache[model_name] = sdf.world.models[0]
  model = model_cache[model_name]

  return model

def marker_msg_to_trimesh(marker_msg):
  mesh = None

  world_transform = tfBuffer.lookup_transform("world",
                                              marker_msg.header.frame_id,
                                              rospy.Time(0), #get the tf at first available time
                                              rospy.Duration(1.0)) #wait for 1 second

  marker_pose = PoseStamped(pose=marker_msg.pose, header=marker_msg.header)
  world_pose = tf2_geometry_msgs.do_transform_pose(marker_pose, world_transform)

  if marker_msg.type == Marker.CUBE:
    extents = (marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z)
    transform = tf.TransformerROS().fromTranslationRotation(
      (world_pose.pose.position.x,
       world_pose.pose.position.y,
       world_pose.pose.position.z),
      (world_pose.pose.orientation.x,
       world_pose.pose.orientation.y,
       world_pose.pose.orientation.z,
       world_pose.pose.orientation.w))
    mesh = trimesh.primitives.Box(extents=extents, transform=transform)
  elif marker_msg.type == Marker.SPHERE:
    radius = marker_msg.scale.x / 2.0
    center = (world_pose.pose.position.x,
              world_pose.pose.position.y,
              world_pose.pose.position.z)
    mesh = trimesh.primitives.Sphere(radius=radius, center=center)
  elif marker_msg.type == Marker.CYLINDER:
    radius = marker_msg.scale.x / 2.0
    height = marker_msg.z
    transform = tf.TransformerROS().fromTranslationRotation(
      (world_pose.pose.position.x,
       world_pose.pose.position.y,
       world_pose.pose.position.z),
      (world_pose.pose.orientation.x,
       world_pose.pose.orientation.y,
       world_pose.pose.orientation.z,
       world_pose.pose.orientation.w))
    mesh = trimesh.primitives.Cylinder(radius=radius, height=height, transform=transform)
  elif marker_msg.type == Marker.MESH_RESOURCE:
    try:
      mesh_path = marker_msg.mesh_resource.replace('file://', '')
      transform = tf.TransformerROS().fromTranslationRotation(
        (world_pose.pose.position.x,
         world_pose.pose.position.y,
         world_pose.pose.position.z),
        (world_pose.pose.orientation.x,
         world_pose.pose.orientation.y,
         world_pose.pose.orientation.z,
         world_pose.pose.orientation.w))
      # Load mesh file as mesh or scene, dump to mesh list & concatenate meshes
      # See: https://github.com/mikedh/trimesh/issues/507#issuecomment-515491388
      mesh_or_scene = trimesh.load(mesh_path)
      dump = mesh_or_scene.dump()
      mesh = dump.sum()
      # Scale mesh
      rospy.logwarn('marker_msg.scale: {}'.format(marker_msg.scale))
      rospy.logwarn('mesh.scale BEFORE mesh.apply_scale(): {}'.format(mesh.scale))
      mesh.apply_scale(np.cbrt(mesh.scale) / mesh.scale)
      rospy.logwarn('mesh.scale AFTER mesh.apply_scale(): {}'.format(mesh.scale))
      # Transform mesh
      mesh.apply_transform(transform)
    except Exception as e:
      rospy.logwarn('gazebo2marker_node: Failed to load mesh {} from marker message: {}'.format(mesh_path, repr(e)))
      pass

  return mesh

def on_model_states_msg(model_states_msg):
  global message_count
  if max_messages <= 0 or message_count < max_messages:
    global lastUpdateTime
    sinceLastUpdateDuration = rospy.get_rostime() - lastUpdateTime
    if sinceLastUpdateDuration.to_sec() < updatePeriod:
      return
    lastUpdateTime = rospy.get_rostime()

    for (model_idx, modelinstance_name) in enumerate(model_states_msg.name):
      try:
        model_name = modelinstance_name
        if (submodelsToBeIncluded and model_name not in submodelsToBeIncluded) or model_name in submodelsToBeIgnored:
          rospy.logdebug("gazebo2marker_node: Model instance {} not included".format(model_name))
          continue
        model = load_model(model_name)
        assert(model)
      except Exception:
        try:
          model_name = pysdf.name2modelname(modelinstance_name)
          if (submodelsToBeIncluded and model_name not in submodelsToBeIncluded) or model_name in submodelsToBeIgnored:
            rospy.logdebug("gazebo2marker_node: Model {} not included".format(model_name))
            continue
          model = load_model(model_name)
          assert(model)
        except Exception as e:
          rospy.logwarn('gazebo2marker_node: Failed to load model {}, '
                        'model instance {}: {}'.format(model_name, modelinstance_name, repr(e)))
          continue
      rospy.logdebug('gazebo2marker_node: Successfully loaded model: {}'.format(modelinstance_name))
      model.for_all_links(publish_link_marker, model_name=model_name, instance_name=modelinstance_name)
    if mesh_output_file:
      try:
        mesh = trimesh.util.concatenate(trimeshes)
        _ = trimesh.exchange.export.export_mesh(mesh, mesh_output_file)
        # scene = trimesh.Scene(trimeshes)
        # _ = trimesh.exchange.export.export_scene(scene, mesh_output_file)
      except Exception as e:
        raise(e)
    message_count += 1
  else:
    return


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-f', '--freq', type=float, default=2, help='Frequency Markers are published (default: 2 Hz)')
  parser.add_argument('-c', '--collision', action='store_true', help='Publish collision instead of visual elements')
  parser.add_argument('-l', '--latch', action='store_true', help='Latch publisher.')
  parser.add_argument('-lf', '--lifetime-forever', action='store_true', help='Set marker message duration lifetime to forever.')
  parser.add_argument('-mm', '--max-messages', type=int, default=-1, help='Maximum number of messages to publish per marker (default: -1 = infinite)')
  parser.add_argument('-w', '--worldfile', type=str, help='Read models from this world file')
  parser.add_argument('-t', '--topic', type=str, default='/visualization_marker', help='Topic to publish markers to')
  parser.add_argument('-mof', '--mesh-output-file', type=str, default=None, help='Export single, unified world mesh to output file (e.g. .stl, .obj)')
  args = parser.parse_args(rospy.myargv()[1:])

  rospy.init_node('gazebo2marker')

  global tfBuffer
  global tfListener
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  global submodelsToBeIncluded
  submodelsToBeIncluded = rospy.get_param('~include_submodels', '').split(';')
  if submodelsToBeIncluded == ['']:
    submodelsToBeIncluded = []
  else:
    rospy.loginfo('gazebo2marker_node: Including submodels: ' + str(submodelsToBeIncluded))

  global submodelsToBeIgnored
  submodelsToBeIgnored = rospy.get_param('~ignore_submodels', '').split(';')
  if submodelsToBeIgnored == ['']:
    submodelsToBeIgnored = []
  else:
    rospy.loginfo('gazebo2marker_node: Ignoring submodels: ' + str(submodelsToBeIgnored))

  global updatePeriod
  updatePeriod = 1. / args.freq

  global use_collision
  use_collision = args.collision

  global latch
  if args.latch:
    latch = True

  global max_messages
  max_messages = args.max_messages

  global lifetime
  if args.lifetime_forever:
    lifetime = rospy.Duration(0)
  else:
    lifetime = rospy.Duration(2 * updatePeriod)

  if args.worldfile:
    global worldsdf
    global submodelsToBeIgnored
    rospy.loginfo('gazebo2marker_node: Loading world model: {}'.format(args.worldfile))
    worldsdf = pysdf.SDF(file=args.worldfile, ignore_submodels=submodelsToBeIgnored)
    if worldsdf:
      rospy.logdebug('gazebo2marker_node: Sucessfully loaded world model {}'.format(args.worldfile))
    else:
      rospy.logwarn('gazebo2marker_node: Failed to load world model {}'.format(args.worldfile))

  global mesh_output_file
  if args.mesh_output_file:
    mesh_output_file = args.mesh_output_file

  global markerPub
  markerPub = rospy.Publisher(args.topic, Marker, queue_size=10, latch=latch)
  rospy.sleep(rospy.Duration(0, 100 * 1000))

  global lastUpdateTime
  lastUpdateTime = rospy.get_rostime()
  modelStatesSub = rospy.Subscriber('gazebo/model_states', ModelStates, on_model_states_msg)

  rospy.loginfo('gazebo2marker_node: Spinning')
  rospy.spin()

if __name__ == '__main__':
  main()
