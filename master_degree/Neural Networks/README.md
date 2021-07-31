# Neural Networks project
This project is an implementation of the paper [Analysis of pedestrian activity before and during COVID-19 lockdown](https://paperswithcode.com/paper/analysis-of-pedestrian-activity-before-and) and it is based on the detections of the YOLO-v3 network.

Please, read this document to understand its structure of directories.

### Download YOLO weights
Please, download the YOLO-v3 weights by executing:
```bash
!wget https://pjreddie.com/media/files/yolov3.weights
```
Then, put them in the 'data' folder.

### Python Notebooks
The main directory provides the following two Python Notebooks:
- 'yolov3_network.ipynb', containing the definition of the model and used to do object detection of input images;
- 'pedestrian_activity.ipynb', to parse the data about the detections of the model.

### Organization of the folders
From the current directory, you are able to see the following folders:
- 'data', with the pre-trained YOLO-v3 weights and all the pre-existing data annotations of the original paper in a csv format;
- 'images_original', that contains ten images provided by the original repository of the paper. It is divided in 'images_input' and 'images_output' folders to distinguish respectively the starting images and the ones elaborated by the model;
- 'model', with just a graphical representation of the network;
- 'results', to group all the output images realized in the 'pedestrian_activity.ipynb' notebook;
- 'images_test', set of images used to test the model and to verify if the original result can be extended to other cities in Europe. 

At the current state of this work, the covered cities are (in alphabetical order):
- Amsterdam
- Barcelona
- Berlin
- London
- Paris
- Rome

Each of these cities creates a sub-directory structured as follows:
- 'data', with a <city_name>.csv file with the detections of the model;
- 'pre_covid', a little set of images dating back to a situation before the Covid-19 spread;
- 'during_covid', a little set of images dating back to a situation during the Covid-19 spread;
- 'results', to group all the outputs of YOLO-v3 on the images in the two previous folders.

## Author
Lorenzo Nicoletti - 1797464
