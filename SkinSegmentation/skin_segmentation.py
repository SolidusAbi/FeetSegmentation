import numpy as np
import cv2
import re, os
from deepgaze.color_detection import RangeColorDetector

subjects_dir = "/home/abian/Workspace/Thesis/Segmentation/DataTest/Images/"
subjects = os.listdir(subjects_dir)

save_dir = "/home/abian/Workspace/Thesis/Segmentation/DataTest/Segmentation/SkinSegmentation/"

# Range from https://arxiv.org/pdf/1708.02694.pdf
min_range = np.array([0, 58, 70], dtype = "uint8") #lower HSV boundary of skin color
max_range = np.array([50, 173, 255], dtype = "uint8") #upper HSV boundary of skin color
my_skin_detector = RangeColorDetector(min_range, max_range) #Define the detector object

regex_pattern = "[^\s]+(RGB_T[0-9]+)(\.(png))$"
r = re.compile(regex_pattern)

for subject in subjects:
    subject_dir = os.path.join(subjects_dir, subject)
    subject_save_dir = os.path.join(save_dir, subject)
    if not os.path.exists(subject_save_dir):
        os.mkdir(subject_save_dir)

    filenames = list(filter(r.match, os.listdir(subject_dir)))

    for filename in filenames:
        current_image_dir = os.path.join(subject_dir, filename)
        image = cv2.imread(current_image_dir)
        mask = my_skin_detector.returnMask(image, morph_opening=True, blur=False, kernel_size=3, iterations=1)
        cv2.imwrite(os.path.join(subject_save_dir, filename), mask) #Save the mask result
    
print("fin")