clear;
clf;
clc;
hold on

%% Loading of all files and bags (Katie) 

tsvFileCheckerBoardLocation = readtable("forCali_GroundTruth_new360.tsv", "FileType","text",'Delimiter', '\t');
bagCalibration = rosbag('CalibNew_360.bag'); 
bagModel = rosbag('Modelnew4_360.bag');


%% Loading of all needed topics (katie)

% topics from calibration bag
RGBimageData = select(bagCalibration, 'Topic', '/camera/color/image_raw');
EMsensorData = select(bagCalibration, 'Topic', '/ndi_aurora');
emData = readMessages(EMsensorData, 'DataFormat', 'struct'); 

% topics from model bag
RGBImages = select(bagModel, 'Topic', '/camera/color/image_raw');
depthImages = select(bagModel, 'Topic', '/camera/depth/image_rect_raw');

EMsensorModel = select(bagModel, 'Topic', '/ndi_aurora');
emDataModel = readMessages(EMsensorModel, 'DataFormat', 'struct'); 


%% Camera Intrinsics (Jesse)

fc = [ 606.909649863209097 ; 606.161507189627287 ];  %-- Focal length:
cc = [ 312.070668018698427 ; 231.994877229425896 ];  %-- Principal point:
alpha_c = 0.000000000000000;  %-- Skew coefficient:
kc = [ 0.123876991636229 ; -0.139744830407497 ; 0.001829407315180 ; -0.004394150159800 ; 0.000000000000000 ];  %-- Distortion coefficients:
fc_error = [ 1.768896888942393 ; 1.700522768808265 ];  %-- Focal length uncertainty:
cc_error = [ 1.750360724117747 ; 1.420781442835046 ];  %-- Principal point uncertainty:
alpha_c_error = 0.000000000000000;  %-- Skew coefficient uncertainty:
kc_error = [ 0.008421053325179 ; 0.037117414381477 ; 0.000991571609629 ; 0.001275086572419 ; 0.000000000000000 ];  %-- Distortion coefficients uncertainty:
nx = 640;  %-- Image size:
ny = 480;

fx = 606.909649863209097;
fy = 606.161507189627287;
px = 312.070668018698427;
py = 231.994877229425896;

K = [fx, 0, px;
     0, fy, py;
     0,  0,  1];


%% Camera Locations in relation to each other from the toolbox calibration (Jesse)

% Creating varibles for storage 
CameraFrameLocations = zeros(3,54);
CameraFrameRotations = zeros(3,54);

%-- Image #1:
CameraFrameRotations(:,1) = [2.318258e+00 ; 1.740305e+00 ; -2.358596e-01];
CameraFrameLocations(:,1)  = [-6.650308e+01 ; -3.110564e+01 ; 2.302668e+02];
%-- Image #2:
CameraFrameRotations(:,2) = [ 2.322686e+00 ; 1.738174e+00 ; -2.313241e-01 ];
CameraFrameLocations(:,2)  = [ -6.240322e+01 ; -3.084272e+01 ; 2.304862e+02 ];
%-- Image #3:
CameraFrameRotations(:,3) = [  2.297692e+00 ; 1.742110e+00 ; -1.358756e-01];
CameraFrameLocations(:,3)  = [ -6.118820e+01 ; -3.630075e+01 ; 2.258658e+02];
%-- Image #4:
CameraFrameRotations(:,4) = [2.287832e+00 ; 1.719329e+00 ; -2.782583e-02 ];
CameraFrameLocations(:,4)  = [ -6.731261e+01 ; -4.133676e+01 ; 2.175506e+02];
%-- Image #5:
CameraFrameRotations(:,5) = [ 2.262243e+00 ; 1.700960e+00 ; -2.171050e-02 ];
CameraFrameLocations(:,5)  = [ -7.416700e+01 ; -3.614532e+01 ; 2.119369e+02 ];
%-- Image #6:
CameraFrameRotations(:,6) = [ 2.200499e+00 ; 1.691339e+00 ; 9.601987e-04 ];
CameraFrameLocations(:,6)  = [ -7.514593e+01 ; -2.275639e+01 ; 2.119614e+02 ];
%-- Image #7:
CameraFrameRotations(:,7) = [ 2.196123e+00 ; 1.670262e+00 ; 7.676922e-02 ];
CameraFrameLocations(:,7)  = [ -7.547397e+01 ; -3.680026e+01 ; 2.057715e+02 ];
%-- Image #8:
CameraFrameRotations(:,8) = [ 2.239398e+00 ; 1.707389e+00 ; -4.269558e-02 ];
CameraFrameLocations(:,8)  = [ -6.518693e+01 ; -3.397298e+01 ; 2.206862e+02 ];
%-- Image #9:
CameraFrameRotations(:,9) = [ 2.377020e+00 ; 1.848222e+00 ; 7.275011e-02 ];
CameraFrameLocations(:,9)  = [ -6.402568e+01 ; -6.424784e+01 ; 2.216512e+02 ];
%-- Image #10:
CameraFrameRotations(:,10) = [ 2.281601e+00 ; 1.869082e+00 ; 5.783770e-02 ];
CameraFrameLocations(:,10)  = [ -8.264954e+01 ; -2.671422e+01 ; 2.319501e+02 ];
%-- Image #11:
CameraFrameRotations(:,11) = [ 2.326549e+00 ; 1.899658e+00 ; 7.977138e-02 ];
CameraFrameLocations(:,11)  = [ -7.781749e+01 ; -3.833672e+01 ; 2.345609e+02 ];
%-- Image #12:
CameraFrameRotations(:,12) = [ -2.414441e+00 ; -1.974305e+00 ; -1.981291e-01 ];
CameraFrameLocations(:,12)  = [ -7.525937e+01 ; -6.916301e+01 ; 2.306098e+02 ];
%-- Image #13:
CameraFrameRotations(:,13) = [ 2.342617e+00 ; 1.952833e+00 ; 1.404958e-01 ];
CameraFrameLocations(:,13)  = [ -8.285492e+01 ; -4.708715e+01 ; 2.406753e+02 ];
%-- Image #14:
CameraFrameRotations(:,14) = [ 2.379279e+00 ; 1.978678e+00 ; 2.092035e-01 ];
CameraFrameLocations(:,14)  = [ -8.535692e+01 ; -5.469884e+01 ; 2.362959e+02 ];
%-- Image #15:
CameraFrameRotations(:,15) = [ 2.326648e+00 ; 1.882551e+00 ; 8.388612e-03 ];
CameraFrameLocations(:,15)  = [ -7.695912e+01 ; -3.739797e+01 ; 2.510252e+02 ];
%-- Image #16:
CameraFrameRotations(:,16) = [ 2.233757e+00 ; 1.811665e+00 ; -1.614263e-01 ];
CameraFrameLocations(:,16)  = [ -6.974734e+01 ; -1.684473e+01 ; 2.634269e+02 ];
%-- Image #17:
CameraFrameRotations(:,17) = [ 2.187806e+00 ; 1.749370e+00 ; -3.179143e-01 ];
CameraFrameLocations(:,17)  = [ -5.760006e+01 ; -1.191110e+01 ; 2.746426e+02 ];
%-- Image #18:
CameraFrameRotations(:,18) = [ 2.163239e+00 ; 1.755077e+00 ; -3.748607e-01 ];
CameraFrameLocations(:,18)  = [ -4.894185e+01 ; -1.340105e+01 ; 2.774333e+02 ];
%-- Image #19:
CameraFrameRotations(:,19) = [ 2.138845e+00 ; 1.699555e+00 ; -4.516473e-01 ];
CameraFrameLocations(:,19)  = [ -3.786247e+01 ; -5.674142e+00 ; 2.799329e+02 ];
%-- Image #20:
CameraFrameRotations(:,20) = [ 2.154435e+00 ; 1.749108e+00 ; -3.582149e-01 ];
CameraFrameLocations(:,20)  = [ -4.797941e+01 ; -1.979502e+01 ; 2.807263e+02 ];
%-- Image #21:
CameraFrameRotations(:,21) = [ 2.163195e+00 ; 1.758219e+00 ; -3.880751e-01 ];
CameraFrameLocations(:,21)  = [ -4.045009e+01 ; -2.012543e+01 ; 2.804704e+02 ];
%-- Image #22:
CameraFrameRotations(:,22) = [ 2.286150e+00 ; 1.856303e+00 ; -4.151762e-01 ];
CameraFrameLocations(:,22)  = [ -4.590658e+01 ; -2.193182e+01 ; 2.886561e+02 ];
%-- Image #23:
CameraFrameRotations(:,23) = [ 2.358987e+00 ; 1.940419e+00 ; -5.055014e-01 ];
CameraFrameLocations(:,23)  = [ -4.344169e+01 ; -3.050221e+01 ; 2.924018e+02 ];
%-- Image #24:
CameraFrameRotations(:,24) = [ -2.386886e+00 ; -1.955604e+00 ; 5.055028e-01 ];
CameraFrameLocations(:,24)  = [ -4.325549e+01 ; -3.678321e+01 ; 2.860606e+02 ];
%-- Image #25:
CameraFrameRotations(:,25) = [ -2.382236e+00 ; -1.956978e+00 ; 5.335828e-01 ];
CameraFrameLocations(:,25)  = [ -5.744404e+01 ; -3.297307e+01 ; 2.902095e+02 ];
%-- Image #26:
CameraFrameRotations(:,26) = [ -2.361084e+00 ; -1.906868e+00 ; 6.151814e-01 ];
CameraFrameLocations(:,26)  = [ -4.773365e+01 ; -2.831663e+01 ; 2.977335e+02 ];
%-- Image #27:
CameraFrameRotations(:,27) = [ -2.344219e+00 ; -1.890282e+00 ; 6.564237e-01 ];
CameraFrameLocations(:,27)  = [-4.391854e+01 ; -3.026317e+01 ; 2.973654e+02 ];
%-- Image #28:
CameraFrameRotations(:,28) = [ -2.340610e+00 ; -1.867681e+00 ; 6.553173e-01 ];
CameraFrameLocations(:,28)  = [ -5.760849e+01 ; -3.187130e+01 ; 2.910635e+02 ];
%-- Image #29:
CameraFrameRotations(:,29) = [ -2.271870e+00 ; -1.840656e+00 ; 6.290540e-01 ];
CameraFrameLocations(:,29)  = [ -5.332886e+01 ; -4.747536e+01 ; 2.917367e+02 ];
%-- Image #30:
CameraFrameRotations(:,30)= [ -2.179776e+00 ; -1.874156e+00 ; 6.348227e-01 ];
CameraFrameLocations(:,30)  = [ -4.406749e+01 ; -5.528697e+01 ; 2.963565e+02 ];
%-- Image #31:
CameraFrameRotations(:,31) = [ -2.132306e+00 ; -1.833370e+00 ; 6.387269e-01 ];
CameraFrameLocations(:,31)  = [ -4.124119e+01 ; -5.097218e+01 ; 3.015757e+02];
%-- Image #32:
CameraFrameRotations(:,32) = [ -2.108572e+00 ; -1.777896e+00 ; 7.018147e-01 ];
CameraFrameLocations(:,32)  = [ -3.463002e+01 ; -4.710033e+01 ; 3.138642e+02];
%-- Image #33:
CameraFrameRotations(:,33) = [ -2.108282e+00 ; -1.765327e+00 ; 7.261420e-01 ];
CameraFrameLocations(:,33)  = [ -3.306383e+01 ; -4.583545e+01 ; 3.161191e+02];
%-- Image #34:
CameraFrameRotations(:,34) = [ -2.081451e+00 ; -1.754202e+00 ; 7.348516e-01 ];
CameraFrameLocations(:,34)  = [ -2.870351e+01 ; -4.604031e+01 ; 3.171038e+02];
%-- Image #35:
CameraFrameRotations(:,35) = [ -2.210596e+00 ; -1.765814e+00 ; 6.115096e-01 ];
CameraFrameLocations(:,35)  = [ -4.754362e+01 ; -4.145954e+01 ; 3.247603e+02];
%-- Image #36:
CameraFrameRotations(:,36) = [ 2.501766e+00 ; 1.876693e+00 ; -3.383776e-01 ];
CameraFrameLocations(:,36)  = [ -7.663179e+01 ; -2.578737e+01 ; 3.039192e+02];
%-- Image #37:
CameraFrameRotations(:,37) = [ 2.345422e+00 ; 1.758873e+00 ; -2.517439e-03 ];
CameraFrameLocations(:,37)  = [ -9.178684e+01 ; -1.977233e+01 ; 2.757471e+02];
%-- Image #38:
CameraFrameRotations(:,38) = [ 2.277840e+00 ; 1.678346e+00 ; 1.760820e-01 ];
CameraFrameLocations(:,38)  = [ -9.469750e+01 ; -1.814778e+01 ; 2.482186e+02];
%-- Image #39:
CameraFrameRotations(:,39) = [ 2.202210e+00 ; 1.653834e+00 ; 2.836146e-01 ];
CameraFrameLocations(:,39)  = [ -9.392024e+01 ; -1.627507e+01 ; 2.244755e+02];
%-- Image #40:
CameraFrameRotations(:,40) = [ 2.141167e+00 ; 1.607525e+00 ; 2.560434e-01 ];
CameraFrameLocations(:,40)  = [ -8.525062e+01 ; 2.207014e+00 ; 2.160933e+02];
%-- Image #41:
CameraFrameRotations(:,41) = [ 2.147196e+00 ; 1.690361e+00 ; 2.869253e-01 ];
CameraFrameLocations(:,41)  = [ -6.948653e+01 ; -2.913632e+01 ; 2.213658e+02];
%-- Image #42:
CameraFrameRotations(:,42) = [ 2.108691e+00 ; 1.659808e+00 ; 3.981610e-01 ];
CameraFrameLocations(:,42)  = [ -7.991577e+01 ; -3.446779e+01 ; 1.939572e+02];
%-- Image #43:
CameraFrameRotations(:,43) = [ 2.029588e+00 ; 1.646300e+00 ; 4.338484e-01 ];
CameraFrameLocations(:,43)  = [ -8.510907e+01 ; -3.082291e+01 ; 1.798122e+02];
%-- Image #44:
CameraFrameRotations(:,44) = [ 2.063912e+00 ; 1.703223e+00 ; 3.756230e-01 ];
CameraFrameLocations(:,44)  = [ -5.673874e+01 ; -2.813037e+01 ; 2.061815e+02 ];
%-- Image #45:
CameraFrameRotations(:,45) = [ 2.033591e+00 ; 1.689187e+00 ; 4.072915e-01 ];
CameraFrameLocations(:,45)  = [ -5.749764e+01 ; -3.585506e+01 ; 1.960327e+02 ];
%-- Image #46:
CameraFrameRotations(:,46) = [ 2.011614e+00 ; 1.664654e+00 ; 4.391212e-01 ];
CameraFrameLocations(:,46)  = [ -6.278900e+01 ; -3.308514e+01 ; 1.841095e+02 ];
%-- Image #47:
CameraFrameRotations(:,47) = [ 2.167429e+00 ; 1.831041e+00 ; 3.397736e-01 ];
CameraFrameLocations(:,47)  = [ -6.015682e+01 ; -4.176332e+01 ; 2.338656e+02 ];
%-- Image #48:
CameraFrameRotations(:,48) = [ 2.346383e+00 ; 2.007596e+00 ; 3.725655e-01 ];
CameraFrameLocations(:,48)  = [ -5.662474e+01 ; -6.514126e+01 ; 2.673293e+02 ];
%-- Image #49:
CameraFrameRotations(:,49) = [ -2.230176e+00 ; -1.974313e+00 ; -4.377531e-01 ];
CameraFrameLocations(:,49)  = [ -6.258700e+01 ; -5.612249e+01 ; 2.550221e+02  ];
%-- Image #50:
CameraFrameRotations(:,50) = [ -2.123049e+00 ; -1.887441e+00 ; -5.203683e-01 ];
CameraFrameLocations(:,50)  = [ -5.587115e+01 ; -5.805193e+01 ; 2.693691e+02  ];
%-- Image #51:
CameraFrameRotations(:,51) = [ -2.066380e+00 ; -1.828098e+00 ; -5.414558e-01 ];
CameraFrameLocations(:,51)  = [ -6.189246e+01 ; -4.539400e+01 ; 2.430539e+02  ];
%-- Image #52:
CameraFrameRotations(:,52) = [ -2.058539e+00 ; -1.901287e+00 ; -5.833852e-01 ];
CameraFrameLocations(:,52)  = [ -8.142574e+01 ; -3.820249e+01 ; 2.441187e+02  ];
%-- Image #53:
CameraFrameRotations(:,53) = [ -2.258668e+00 ; -2.041189e+00 ; -2.448459e-01 ];
CameraFrameLocations(:,53)  = [ -5.057474e+01 ; -1.864514e+01 ; 2.876224e+02  ];
%-- Image #54:
CameraFrameRotations(:,53) = [  -2.338344e+00 ; -2.046368e+00 ; -1.495020e-01 ];
CameraFrameLocations(:,53)  = [ -3.538009e+01 ; -1.881138e+01 ; 2.957791e+02  ];


% Stores all the locations in a transformation matrix
cameraFrameMatricies = createMatrices(CameraFrameRotations, CameraFrameLocations);


%% Tsv file location plotted for refrence (Jesse)

% this code iterates over all the points and saves the data to two seperate
% variables 
for i = 1:70
    hold on
    SingularCheckerboardFeatureLocations = tsvFileCheckerBoardLocation(i,10:12);
    TableCheckerboardFeatureTranslations(i, :) = SingularCheckerboardFeatureLocations;
    SingularCheckerboardFeatureLocations = tsvFileCheckerBoardLocation(i,7:9);
    TableCheckerboardFeatureRotations(i,:) =  SingularCheckerboardFeatureLocations;
end

% converts the tables to varibales that can be plotted 
CheckerboardFeatureTranslations = table2array(TableCheckerboardFeatureTranslations);
CheckerboardFeatureRotations = table2array(TableCheckerboardFeatureRotations);

% plots the variables in a point cloud
ptCloud = pointCloud(CheckerboardFeatureTranslations,'Color',[1,0,0]);
pcshow(ptCloud);
title('Checkerboard Feature Point Cloud');
xlabel('X');
ylabel('Y');
zlabel('Z');

% puts the points into a matrix array to easily access and view
CheckerboardFeatureTranslations = CheckerboardFeatureTranslations';
CheckerboardFeatureRotations = CheckerboardFeatureRotations';
CheckerBoardMatricies = createMatrices(CheckerboardFeatureRotations,CheckerboardFeatureTranslations);

%% EM sensor data plotting Calibration Bag (Jesse and Katie)

% the code goes through all of the em sensor data and takes every tenth
% value and stores it 
for i =1:10:min(564, numel(emData))
    em_sensor = emData{i};
    Translationx = em_sensor.PortHandlePoses.Translation.X;
    Translationy = em_sensor.PortHandlePoses.Translation.Y;
    Translationz = em_sensor.PortHandlePoses.Translation.Z;

    Rotationx = em_sensor.PortHandlePoses.Rotation.X;
    Rotationy = em_sensor.PortHandlePoses.Rotation.Y;
    Rotationz = em_sensor.PortHandlePoses.Rotation.Z;

    % Store the XYZ Translation data in EmSensorMatrix
    EmSensorTranlation(1, i) = Translationx;
    EmSensorTranlation(2, i) = Translationy;
    EmSensorTranlation(3, i) = Translationz;

    % Store the XYZ Rotation Data in
    EmSensorRotation(1, i) = Rotationx;
    EmSensorRotation(2, i) = Rotationy;
    EmSensorRotation(3, i) = Rotationz;

    end

% this code just ignores any zero values as they arent important 
exceptionRow = 181;
EmSensorRotation = EmSensorRotation';
rowsWithZeros = any(EmSensorRotation == 0, 2);
rowsWithZeros(exceptionRow) = false;
EmSensorRotation = EmSensorRotation(~rowsWithZeros, :);
EmSensorRotation = EmSensorRotation';

EmSensorTranlation = EmSensorTranlation';
rowsWithZeros = any(EmSensorTranlation == 0, 2);
rowsWithZeros(exceptionRow) = false;
EmSensorTranlation = EmSensorTranlation(~rowsWithZeros, :);
EmSensorTranlation = EmSensorTranlation';

% stores all the values into a matrix array so they are easy to access
EmSensorMatricies = createMatrices(EmSensorRotation,EmSensorTranlation);

% this code plots the xyz values in a point cloud to visualise what is
% happening 
ptCloud = pointCloud(EmSensorTranlation', 'Color',[0,1,0]);
pcshow(ptCloud);
title('Camera In realtion to CheckerBoard (Calibration Bag)');
xlabel('X');
ylabel('Y');
zlabel('Z');


%% plots the camera frame matricies (Jesse)

figure(2)
ptCloud1 = pointCloud(CameraFrameLocations','Color',[0,1,1]);
pcshow(ptCloud1);
title('Camera Frames in realation to each other');
xlabel('X');
ylabel('Y');
zlabel('Z');


%% Finding timeStamps to pair data for Calibration Bag (Jesse)

% this code looks at the nanosecond time stamp within the image data and
% then converts it to a date and time in relation to epoch so the correct
% images were used with the correct data
for i = 1:30:RGBimageData.NumMessages
RGBImage = readMessages(RGBimageData,i);
ros_timestamp_nanoseconds = RGBImage{1}.Header.Stamp.Sec;
seconds_since_epoch = floor(ros_timestamp_nanoseconds);
nanoseconds = mod(ros_timestamp_nanoseconds, 1e9);
reference_epoch = datetime('1970-01-01 00:00:00', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'Format', 'd-MMM-y HH:mm:ss.SSSSSSSSS');
timeStampsRGBImages_Calibration(i,:) = reference_epoch + seconds(seconds_since_epoch) + milliseconds(nanoseconds / 1e6);
end


for i =1:10:min(564, numel(emData))
ros_timestamp_nanoseconds = emData{i}.Header.Stamp.Sec;
    seconds_since_epoch = floor(ros_timestamp_nanoseconds);
    nanoseconds = mod(ros_timestamp_nanoseconds, 1e9);
    reference_epoch = datetime('1970-01-01 00:00:00', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'Format', 'd-MMM-y HH:mm:ss.SSSSSSSSS');
    timeStampsEmSensor(i,:) = reference_epoch + seconds(seconds_since_epoch) + milliseconds(nanoseconds / 1e6);
end

%% using selvester equation to obtain pose between camera and em sensor (Jesse)

% this code finds the transform between two camera frames and two em sensor
% locations
A = inv(cameraFrameMatricies{1}) * cameraFrameMatricies{2}; 
B = inv(EmSensorMatricies{1}) * EmSensorMatricies{2};

C = eye(4);

% this code then attempts to find the realative pose between the camera and
% em sensor 
X = sylvester(A,B,C);


%% Finding timestapes to pair data for Model Bag (Luca)


% this code looks at the nanosecond time stamp within the image data and
% then converts it to a date and time in relation to epoch so the correct
% images were used with the correct data
for i = 1:30:1200 %(this number was chosen because of timestamps) (there is also 40 rgb images)
    RGBImage = readMessages(RGBImages,i);
    ros_timestamp_nanoseconds = RGBImage{1}.Header.Stamp.Sec;
    seconds_since_epoch = floor(ros_timestamp_nanoseconds);
    nanoseconds = mod(ros_timestamp_nanoseconds, 1e9);
    reference_epoch = datetime('1970-01-01 00:00:00', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'Format', 'd-MMM-y HH:mm:ss.SSSSSSSSS');
    timeStampsRGBImages_Model(i,:) = reference_epoch + seconds(seconds_since_epoch) + milliseconds(nanoseconds / 1e6);
end

for i = 1:4:depthImages.NumMessages %(there are 40 depth images)
    depthImage = readMessages(depthImages,i);
    ros_timestamp_nanoseconds = depthImage{1}.Header.Stamp.Sec;
    seconds_since_epoch = floor(ros_timestamp_nanoseconds);
    nanoseconds = mod(ros_timestamp_nanoseconds, 1e9);
    reference_epoch = datetime('1970-01-01 00:00:00', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'Format', 'd-MMM-y HH:mm:ss.SSSSSSSSS');
    timeStampsDepthImages_Model(i,:) = reference_epoch + seconds(seconds_since_epoch) + milliseconds(nanoseconds / 1e6);
end


for i =1:2:min(564, numel(emDataModel))
ros_timestamp_nanoseconds = emDataModel{i}.Header.Stamp.Sec;
    seconds_since_epoch = floor(ros_timestamp_nanoseconds);
    nanoseconds = mod(ros_timestamp_nanoseconds, 1e9);
    reference_epoch = datetime('1970-01-01 00:00:00', 'InputFormat', 'yyyy-MM-dd HH:mm:ss', 'Format', 'd-MMM-y HH:mm:ss.SSSSSSSSS');
    timeStampsEmSensor_Model(i,:) = reference_epoch + seconds(seconds_since_epoch) + milliseconds(nanoseconds / 1e6);
end

%% EM sensor data plotting Model Bag (Katie and Luca)

% the code goes through all of the em sensor data and takes every second
% value and stores it  
for i =1:2:min(564, numel(emDataModel))
    em_sensor_model = emDataModel{i};
    Translationx_model = em_sensor_model.PortHandlePoses.Translation.X;
    Translationy_model = em_sensor_model.PortHandlePoses.Translation.Y;
    Translationz_model = em_sensor_model.PortHandlePoses.Translation.Z;

    Rotationx_model = em_sensor_model.PortHandlePoses.Rotation.X;
    Rotationy_model = em_sensor_model.PortHandlePoses.Rotation.Y;
    Rotationz_model = em_sensor_model.PortHandlePoses.Rotation.Z;

    % Store the XYZ Translation data in EmSensorMatrix
    EmSensorTranlation_model(1, i) = Translationx_model;
    EmSensorTranlation_model(2, i) = Translationy_model;
    EmSensorTranlation_model(3, i) = Translationz_model;

    % Store the XYZ Rotation Data in
    EmSensorRotation_model(1, i) = Rotationx_model;
    EmSensorRotation_model(2, i) = Rotationy_model;
    EmSensorRotation_model(3, i) = Rotationz_model;

    end

% converts the data to a matrix array 
EmSensorMatricies_model = createMatrices(EmSensorRotation_model,EmSensorTranlation_model);

% this then plots the points in a point cloud 
ptCloud = pointCloud(EmSensorTranlation_model', 'Color',[0,1,0]);
pcshow(ptCloud);
title('Camera In realtion to CheckerBoard (Model Bag)');
xlabel('X');
ylabel('Y');
zlabel('Z');


%% Putting images in Arrays For Plotting (Katie)

% this code then buts the correct depth and rgb images into arrays so that
% they can eassily be accessed later in the code
index = 1;
for i = 1:30:1200
    RGBImage = readMessages(RGBImages,i);
    rgb_img_Array{index} = readImage(RGBImage{1});
    index = index + 1;
end

index = 1;
for  i = 1:4:depthImages.NumMessages
    DepthImage = readMessages(depthImages,i);
    depth_img = readImage(DepthImage{1});
    depth_img =  uint8(double(depth_img)/ 2300*255);
    depth_img_Array{index} = imresize(depth_img,[480 640]);
    index = index + 1;
end


%% displaying output attempt 1 and 2 (Jesse and Luca)


% this code iterates over all the pixils in the depth and rgb images to get
% the correct variables to then plot for the final outputs 
for i = 1:40

    index = 1;
    for u = 1 : 480
        for v = 1 : 640

            depthValue = depth_img_Array{i}(u,v);

            if depthValue > 35
                depthValue = 35;
            end

            X_cam = [(u - px) / fx; (v - py) / fy; 1];
            X_cam(3,1) = X_cam(3,1) * depthValue;

            CorrdinateValues(:,index) = X_cam ;
            R_values(index) = rgb_img_Array{i}(u, v, 1);
            G_values(index) = rgb_img_Array{i}(u, v, 2);
            B_values(index) = rgb_img_Array{i}(u, v, 3);


            index = index + 1;

        end
    end

    RGBValues(1,:) = R_values;
    RGBValues(2,:) = G_values;
    RGBValues(3,:) = B_values;

    ptClouds{i} = pointCloud(CorrdinateValues',"Color",RGBValues');

    if i == 1
        ptCloud1 = pointCloud(CorrdinateValues',"Color",RGBValues');
    end

    figure(4)
    subplot(4,10,i)
    ptCloud = pointCloud(CorrdinateValues',"Color",RGBValues');
    pcshow(ptCloud);
    title(['Image With Depth',num2str(i)])

    figure(5)
    pcshow(ptCloud)
    title('Final output Attempt_1')

    figure(6)
    ptCloudMerged = pcmerge(ptCloud1,ptCloud,1);
    pcshow(ptCloudMerged)
    title('Final output Attempt_2')
end

%% display output attempt 3 (Jesse)

% this is the final attempt at trying to plot a final output by using icp
% to merge the point clouds 
ptCloudRef = ptClouds{1};
ptCloudCurrent = ptClouds{2};

gridSize = 0.1;
fixed = pcdownsample(ptCloudRef,gridAverage=gridSize);
moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);

tform = pcregistericp(moving,fixed);
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
ptCloudScene1 = pcmerge(ptCloudRef,ptCloudAligned,mergeSize);

accumTform = tform; 

figure(7)
hAxes = pcshow(ptCloudScene1,VerticalAxis="Y",VerticalAxisDir="Down");
title("final output attempt_3")
xlabel("X (m)")
ylabel("Y (m)")
zlabel("Z (m)")

% Set the axes property for faster rendering.
hAxes.CameraViewAngleMode = "auto";
hScatter = hAxes.Children;

for i = 3:length(ptClouds)
    ptCloudCurrent = ptClouds{i};
       
    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent,gridAverage=gridSize);
    
    % Apply ICP registration.
    tform = pcregistericp(moving,fixed);

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = rigidtform3d(accumTform.A * tform.A);
    ptCloudAligned = pctransform(ptCloudCurrent,accumTform);
    
    % Update the world scene.
    ptCloudScene1 = pcmerge(ptCloudScene1,ptCloudAligned,mergeSize);

    % Visualize the world scene.
    hScatter.XData = ptCloudScene1.Location(:,1);
    hScatter.YData = ptCloudScene1.Location(:,2);
    hScatter.ZData = ptCloudScene1.Location(:,3);
    hScatter.CData = ptCloudScene1.Color;
    drawnow("limitrate")
end


%% Clearing all temporary valiables to clean workspace (comment out to see all variables) (Jesse)

clear("A","accumTform","alpha_c","alpha_c_error","B","B_values","C", ...
    "CameraFrameLocations","CameraFrameRotations","cc","cc_error", ...
    "CheckerboardFeatureRotations", "CheckerboardFeatureTranslations", ...
    "CorrdinateValues","depth_img","depthImage","DepthImage","depthValue" ...
    ,"em_sensor","em_sensor_model","emData","emDataModel","EmSensorRotation" ...
    ,"EmSensorRotation_model","EmSensorTranlation","EmSensorTranlation_model" ...
    ,"exceptionRow","fc","fc_error","fx","fy","G_values","fixed","gridSize", ...
    "hAxes","hScatter","i","index","hScatter","kc","kc_error","mergeSize","moving", ...
    "nanoseconds","nx","ny","ptCloud","ptCloud","ptCloudAligned","ptCloudCurrent" ...
    ,"ptCloudMerged","ptCloudRef","ptCloudScene1","px","py","R_values", ...
    "reference_epoch","RGBImage","ros_timestamp_nanoseconds","Rotationx", ...
    "Rotationx_model","Rotationy","Rotationy_model","Rotationz","Rotationz_model" ...
    ,"rowsWithZeros","seconds_since_epoch","Translationx","Translationx_model", ...
    "Translationy","Translationy_model","Translationz","Translationz_model","u" ...
    ,"v","X_cam")





%% Function used to create matricies (Jesse)

function transformation_matrices = createMatrices(Rotation_values, Translation_values)
    num_poses = size(Rotation_values, 2);
    transformation_matrices = cell(1, num_poses);

    for i = 1:num_poses
        Rotation = Rotation_values(:, i);
        Translation = Translation_values(:, i);
        
        % Create the 3x3 rotation matrix from omc
        % R = rodrigues(Rotation);
  
         R = rotvec2mat3d(Rotation);
        % Create a 4x4 transformation matrix
        T = eye(4);
        T(1:3, 1:3) = R;
        T(1:3, 4) = Translation;
        
        % Store the transformation matrix in the cell array
        transformation_matrices{i} = T;
    end
end

%% This was used to get the checkerboard images for calibration (Jesse)
% start = 1;
% step = 30;
% stop = 1627;
% imageArray = cell(1, ceil((stop - start) / step));
% index = 1;
% 
% for i = start:step:stop
%     figure(i);
%     RGBImage = readMessages(RGBimageData, i);
%     rgb_img = readImage(RGBImage{1});
%     imageArray{index} = rgb_img;
%     filename = sprintf('CalibrationImage_%d.jpg', index);
%     imwrite(rgb_img, filename, 'jpg');
%     % imshow(rgb_img)
%     index = index + 1;
% end






