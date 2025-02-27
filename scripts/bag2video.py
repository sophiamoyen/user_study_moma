import rosbag
import cv2
from cv_bridge import CvBridge
import argparse
import glob

def create_video(bag_file, image_topic, output_video):
    bridge = CvBridge()
    bag = rosbag.Bag(bag_file, 'r')

    # Define the codec and video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    frame_rate = 10  # You can adjust this as needed
    frame_size = None  # Will be determined from the first frame

    video_writer = None

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"Error converting message: {e}")
            continue

        if frame_size is None:
            frame_size = (frame.shape[1], frame.shape[0])
            video_writer = cv2.VideoWriter(output_video, fourcc, frame_rate, frame_size)

        video_writer.write(frame)

    bag.close()

    if video_writer is not None:
        video_writer.release()
        print(f"Video saved as {output_video}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert ROS1 bag image topic to MP4 video")
    parser.add_argument("bag_files", nargs="+", help="Paths to the ROS1 bag files (supports wildcards like *.bag)")
    parser.add_argument("image_topic", help="Image topic to extract from the bag file")
    parser.add_argument("output_directory", help="Directory to save the output MP4 videos")
    args = parser.parse_args()

    for bag_file in args.bag_files:
        bag_paths = glob.glob(bag_file)
        for path in bag_paths:
            output_video = f"{args.output_directory}/{path.split('/')[-1].replace('.bag', '.mp4')}"
            create_video(path, args.image_topic, output_video)