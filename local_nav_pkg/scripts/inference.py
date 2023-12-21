#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

from xml.etree.ElementTree import tostring
import rclpy
#import rospy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from local_nav_pkg.msg import PenguinInference
from local_nav_pkg.msg import PenguinInferenceList
from local_nav_pkg.msg import Corner
from sensor_msgs.msg import Image as ImageMessage
from cv_bridge import CvBridge  

import tensorflow as tf # import tensorflow
import numpy as np
from PIL import Image
from matplotlib import pyplot as plt
from tqdm import tqdm
import sys # importyng sys in order to access scripts located in a different folder
import os # importing OS in order to make GPU visible

os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID" # do not change anything in here

# specify which device you want to work on.
# Use "-1" to work on a CPU. Default value "0" stands for the 1st GPU that will be used
os.environ["CUDA_VISIBLE_DEVICES"]="0" # TODO: specify your computational device

PUBLISH_IMAGE = True
USE_STATIC = False
TO_FILE = False


# checking that GPU is found
if tf.test.gpu_device_name():
    print('GPU found')
else:
    print("No GPU found")

# other import
new_image = [[[0, 0, 0]]]
path2scripts = '/home/administrator/TensorFlow/models/research/' # TODO: provide pass to the research folder
sys.path.insert(0, path2scripts) # making scripts in models/research available for import

# importing all scripts that will be needed to export your model and use it for inference
from object_detection.utils import label_map_util
from object_detection.utils import config_util
from object_detection.utils import visualization_utils as viz_utils
from object_detection.builders import model_builder

# NOTE: your current working directory should be Tensorflow.

# TODO: specify two pathes: to the pipeline.config file and to the folder with trained model.
path2config ='/home/administrator/TensorFlow/penguin_detection_model/exported_models/eff_det_3_23_3/pipeline.config'
path2model = '/home/administrator/TensorFlow/penguin_detection_model/exported_models/eff_det_3_23_3/checkpoint'

# do not change anything in this cell
configs = config_util.get_configs_from_pipeline_file(path2config) # importing config
model_config = configs['model'] # recreating model config
detection_model = model_builder.build(model_config=model_config, is_training=False) # importing model

ckpt = tf.compat.v2.train.Checkpoint(model=detection_model)
ckpt.restore(os.path.join(path2model, 'ckpt-0')).expect_partial()

path2label_map = '/home/administrator/TensorFlow/penguin_detection_model/training/object-detection.pbtxt' # TODO: provide a path to the label map file
category_index = label_map_util.create_category_index_from_labelmap(path2label_map,use_display_name=True)

class PenguinInferenceNode(Node):

    def __init__(self):
        super().__init__('penguin_inference_publisher')
        self.subscriber = self.create_subscription(ImageMessage, 'image_raw', self.image_callback, 1)
        
        self.publisher_ = self.create_publisher(PenguinInferenceList, 'inference', 1)
        self.image_publisher_ = self.create_publisher(ImageMessage, 'annotated_image', 1)

        self.penguin_inference_timer_ = self.create_timer(1, self.publish_inference)

    def image_callback(self, msg):
        global new_image
        bridge = CvBridge()
        new_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
    def publish_inference(self):
        path2images = {'/home/administrator/TensorFlow/penguin_detection_model/images/R0010003_st_Moment(23).jpg'}
        #path2images = {'/home/administrator/Desktop/test.jpg'}
        inference_list, detections, num_detections = self.inference_as_raw_output(path2images)
        if PUBLISH_IMAGE:
            annotated_image = self.inference_with_plot(path2images, detections, num_detections)
            self.image_publisher_.publish(annotated_image)
        self.publisher_.publish(inference_list) 

    def inference_with_plot(self, path2images, detections, num_detections, box_th=0.25):
        for image_path in path2images:

            if USE_STATIC:    
                image_np = self.load_image_into_numpy_array(image_path)
            
            #self.get_logger().info('test1')
            #self.get_logger().info(str(np.shape(image_np)))
            #self.get_logger().info('\n')
            
            if not USE_STATIC:
                image_np = np.array(new_image)
            
            #input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)
            #detections = self.detect_fn(input_tensor)

            # All outputs are batches tensors.
            # Convert to numpy arrays, and take index [0] to remove the batch dimension.
            # We're only interested in the first num_detections.
            #num_detections = int(detections.pop('num_detections'))
            #detections = {key: value[0, :num_detections].numpy()
            #                for key, value in detections.items()}
            
            #detections['num_detections'] = num_detections

            # detection_classes should be ints.
            #detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

            label_id_offset = 1
            image_np_with_detections = image_np.copy()

            viz_utils.visualize_boxes_and_labels_on_image_array(
                    image_np_with_detections,
                    detections['detection_boxes'],
                    detections['detection_classes']+label_id_offset,
                    detections['detection_scores'],
                    category_index,
                    use_normalized_coordinates=True,
                    max_boxes_to_draw=200,
                    min_score_thresh=box_th,
                    agnostic_mode=False,
                    line_thickness=5)
            
            bridge = CvBridge()
            try:
                output_image = bridge.cv2_to_imgmsg(image_np_with_detections, encoding='rgb8')
            
            except KeyError:
                output_image = bridge.cv2_to_imgmsg(np.array([[[0.0, 0.0, 0.0]]]))
            # plt.figure(figsize=(15,10))
            # plt.imshow(image_np_with_detections)
            # plt.savefig('toot.png')
            # plt.figure(figsize=(15,10))
            # plt.imshow(image_np)
            # plt.savefig('toottoot.png')
            #print('Done')
            return output_image
            
            

    def detect_fn(self, image):
        """
        Detect objects in image.
        
        Args:
        image: (tf.tensor): 4D input image
        
        Returns:
        detections (dict): predictions that model made
        """

        image, shapes = detection_model.preprocess(image)
        prediction_dict = detection_model.predict(image, shapes)
        detections = detection_model.postprocess(prediction_dict, shapes)

        return detections

    def load_image_into_numpy_array(self, path):
        """Load an image from file into a numpy array.

        Puts image into numpy array to feed into tensorflow graph.
        Note that by convention we put it into a numpy array with shape
        (height, width, channels), where channels=3 for RGB.

        Args:
        path: the file path to the image

        Returns:
        numpy array with shape (img_height, img_width, 3)
        """
        img = Image.open(path)
        new_img = img.resize((1920, 960))
        #self.get_logger().info(str(new_img.size))
        return np.array(new_img)
        
        
    def nms(self, rects, thd=0.5):
        """
        Filter rectangles
        rects is array of oblects ([x1,y1,x2,y2], confidence, class)
        thd - intersection threshold (intersection divides min square of rectange)
        """
        out = []

        remove = [False] * len(rects)

        for i in range(0, len(rects) - 1):
            if remove[i]:
                continue
            inter = [0.0] * len(rects)
            for j in range(i, len(rects)):
                if remove[j]:
                    continue
                inter[j] = self.intersection(rects[i][0], rects[j][0]) / min(self.square(rects[i][0]), self.square(rects[j][0]))

            max_prob = 0.0
            max_idx = 0
            for k in range(i, len(rects)):
                if inter[k] >= thd:
                    if rects[k][1] > max_prob:
                        max_prob = rects[k][1]
                        max_idx = k

            for k in range(i, len(rects)):
                if (inter[k] >= thd) & (k != max_idx):
                    remove[k] = True

        for k in range(0, len(rects)):
            if not remove[k]:
                out.append(rects[k])

        boxes = [box[0] for box in out]
        scores = [score[1] for score in out]
        classes = [cls[2] for cls in out]
        return boxes, scores, classes


    def intersection(self, rect1, rect2):
        """
        Calculates square of intersection of two rectangles
        rect: list with coords of top-right and left-boom corners [x1,y1,x2,y2]
        return: square of intersection
        """
        x_overlap = max(0, min(rect1[2], rect2[2]) - max(rect1[0], rect2[0]));
        y_overlap = max(0, min(rect1[3], rect2[3]) - max(rect1[1], rect2[1]));
        overlapArea = x_overlap * y_overlap
        return overlapArea


    def square(self, rect):
        """
        Calculates square of rectangle
        """
        return abs(rect[2] - rect[0]) * abs(rect[3] - rect[1])
        
    def inference_as_raw_output(self, path2images,
                                box_th = 0.80,
                                nms_th = 0.01,
                                data = "penguin",
                                path2dir = False):
        
        com_list = []
        final_inference_list = PenguinInferenceList()
        inference_list = []
        
        """
        Function that performs inference and return filtered predictions
        
        Args:
        path2images: an array with pathes to images
        box_th: (float) value that defines threshold for model prediction. Consider 0.25 as a value.
        nms_th: (float) value that defines threshold for non-maximum suppression. Consider 0.5 as a value.
        to_file: (boolean). When passed as True => results are saved into a file. Writing format is
        path2image + (x1abs, y1abs, x2abs, y2abs, score, conf) for box in boxes
        data: (str) name of the dataset you passed in (e.g. test/validation)
        path2dir: (str). Should be passed if path2images has only basenames. If full pathes provided => set False.
        
        Returns:
        detections (dict): filtered predictions that model made
        """
        
        for image_path in tqdm(path2images):
            
            if USE_STATIC:
                if path2dir: # if a path to a directory where images are stored was passed in
                    image_path = os.path.join(path2dir, image_path.strip())
                    
                image_np = self.load_image_into_numpy_array(image_path)
            
            #self.get_logger().info('test1')
            #self.get_logger().info(str(np.shape(image_np)))
            #self.get_logger().info('\n')
            
            if not USE_STATIC:
                image_np = np.array(new_image)

            #self.get_logger().info('test2')
            #self.get_logger().info(str(np.shape(image_np)))
            #self.get_logger().info('\n')

            input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)
            detections = self.detect_fn(input_tensor)
            

            # checking how many detections we got
            num_detections = int(detections.pop('num_detections'))
            
            # filtering out detection in order to get only the one that are indeed detections
            detections = {key: value[0, :num_detections].numpy() for key, value in detections.items()}
            
            # detection_classes should be ints.
            detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
                        # defining what we need from the resulting detection dict that we got from model output
            key_of_interest = ['detection_classes', 'detection_boxes', 'detection_scores']
            
            # filtering out detection dict in order to get only boxes, classes and scores
            detections = {key: value for key, value in detections.items() if key in key_of_interest}
            

            if box_th: # filtering detection if a confidence threshold for boxes was given as a parameter
                for key in key_of_interest:
                    scores = detections['detection_scores']
                    current_array = detections[key]
                    filtered_current_array = current_array[scores > box_th]
                    detections[key] = filtered_current_array
            
            #self.get_logger().info('test1')
            #self.get_logger().info(str(type(detections['detection_classes'])))
            detections_raw = detections.copy()
            
            if nms_th: # filtering rectangles if nms threshold was passed in as a parameter
                # creating a zip object that will contain model output info as
                output_info = list(zip(detections['detection_boxes'],
                                    detections['detection_scores'],
                                    detections['detection_classes']
                                    )
                                )
                boxes, scores, classes = self.nms(output_info)
                
                detections['detection_boxes'] = np.array(boxes) # format: [y1, x1, y2, x2]
                detections['detection_scores'] = np.array(scores)
                detections['detection_classes'] = np.array(classes)
            
            detections_raw = detections.copy()    
            
            image_h, image_w, _ = image_np.shape
            if TO_FILE:
                file_name = f'pred_result_{data}.txt'
            
                line2write = list()
                line2write.append(os.path.basename(image_path))
            
                with open(file_name, 'a+') as text_file:
                    # iterating over boxes
                    for b, s, c in zip(boxes, scores, classes):
                        inference = PenguinInference()
                        
                        y1abs, x1abs = b[0] * image_h, b[1] * image_w
                        y2abs, x2abs = b[2] * image_h, b[3] * image_w

                        comx = (x1abs+x2abs)/2
                        comy = (y1abs+y2abs)/2
                        com = comx, comy
                        com_list.append(com)
                        print(com)
                        
                        list2append = [x1abs, y1abs, x2abs, y2abs, s, c]
                        line2append = ','.join([str(item) for item in list2append])
                        
                        line2write.append(line2append)

                    line2write = ' '.join(line2write)
                    text_file.write(line2write + os.linesep)
            
            for b, s, c in zip(boxes, scores, classes):
                #self.get_logger().info(str(s))
                inference = PenguinInference()
                
                y1abs, x1abs = b[0] * 100, b[1] * 100
                y2abs, x2abs = b[2] * 100, b[3] * 100
                
                box = Corner()
                box.corner1 = [x1abs, y1abs]
                box.corner2 = [x1abs, y2abs]
                box.corner3 = [x2abs, y2abs]
                box.corner4 = [x2abs, y1abs]
                inference.box = box

                inference.confidence = float(s)
                inference.type = int(c)
                inference_list.append(inference)      
            final_inference_list.penguin_inference = inference_list
            return final_inference_list, detections_raw, num_detections


def main(args=None):
    rclpy.init(args=args)
    node = PenguinInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()
