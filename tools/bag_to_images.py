"""A script to convert a ROS bag image topic to images with timestamp as names."""

import argparse
import rosbag
from tqdm import tqdm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
# from src.ros_utils import get_camera_image
import sys
import os

del sys.path[1]
import cv2


def bag_to_images(bag: rosbag.Bag, output_file: str, topic: str, timestamp: str):
      """
      Convert a ROS bag with image topic to images with timestamp as names.
      
      Args:
            bag: the bag file to get image data from
            output_file: the path to a directory to write images to
            topic: the topic to read image data from
      
      Returns:
            None
      
      """
      # create a bridge to convert ROS messages to OpenCV images
      bridge = CvBridge()
      # get the total number of frames to write
      total_frames = bag.get_message_count(topic_filters=topic)
      print(f"Total frames: {total_frames} in {topic}")
      # get an iterator for the topic with the frame data
      iterator = bag.read_messages(topics=topic)
      # check if the output directory exists
      if not os.path.isdir(output_file):
            # create the output directory
            os.makedirs(output_file)
      # iterate over the image messages of the given topic
      i = 0
      f =  open(timestamp, 'w')
      for _, msg, _ in tqdm(iterator, total=total_frames):
            # read the image data into a NumPy tensor
            cv_image = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            ts = "%.6f" % msg.header.stamp.to_sec()
            name = str(i) + '.png'
            f.write(ts + ' ' + name +'\n')
            # img = get_camera_image(msg.data, (msg.height, msg.width))
            # write the image to the video file
            cv2.imwrite(os.path.join(output_file, name), cv_image)
            i += 1
      f.close()


if __name__ == '__main__':
      parser = argparse.ArgumentParser(description='Convert a ROS bag with image topic to images with timestamp as names.')
      parser.add_argument('--bag_file', type=str, help='the bag file to get image data from')
      parser.add_argument('--output_dir', type=str, help='the path to a directory to write images to')
      parser.add_argument('--topic', type=str, help='the topic to read image data from')
      parser.add_argument('--timestamp', type=str, help='the timestamp to save image data from')
      args = parser.parse_args()
      # open the bag file
      bag = rosbag.Bag(args.bag_file)
      # convert the bag file to images
      bag_to_images(bag, args.output_dir, args.topic, args.timestamp)
      # close the bag file
      bag.close()
      print('Convert Done!')