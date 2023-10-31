# Sensors-and-Control-Assessment-Submission-

this is our group submission for the project:

Group Members: Jesse Gonzalez - 14259360 Katie Forster - 14242630 Luca Roncolato - 13611088

Project 5: 3D reconstruction using RGB-D camera and EM sensor


The code provided attempts to reconstruct a 3D environment from a set of 2D images with the use of depth images. All data sets were provided by the tutors which included two ROS bags, one including calibration data and the other included the data for the model that needed to be reconstructed. The main process of the code extracts all necessary data and then attempts to find the relative pose between the camera and em senor. The next stage then takes the images from the model bag and then iterates over all the pixels from both the rgb images as well as the depth images. Code then takes the 2D images and plots them in a 3D space through the inverse of the camera matrix K. Once this was completed there were a few attempts using a few different methods to try and recreate the 3D environment. 

To use the code the following files will need to be added to the path and downloaded by the marker as they were too big too be uploaded to github
- CalibNew_360.bag
- forCali_GroundTruth_new360.tsv
- Modelnew4_360.bag

These files can be downloaded from the folling Google Drive
https://drive.google.com/drive/folders/1Ej7wju_iqwBiBit7LN3MVIqnT5xflACx?usp=sharing
