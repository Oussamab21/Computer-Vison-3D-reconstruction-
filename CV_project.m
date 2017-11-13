% Extrinsic parameters between RGB and Depth camera for Kinect V1
% Rotation matrix
R =  inv([  9.9998579449446667e-01, 3.4203777687649762e-03, -4.0880099301915437e-03;
    -3.4291385577729263e-03, 9.9999183503355726e-01, -2.1379604698021303e-03;
    4.0806639192662465e-03, 2.1519484514690057e-03,  9.9998935859330040e-01]);

% Translation vector.
T = -[  2.2142187053089738e-02, -1.4391632009665779e-04, -7.9356552371601212e-03 ]';

% Kinect Depth camera parameters
fx_d =  5.7616540758591043e+02;
fy_d = 5.7375619782082447e+02;
cx_d = 3.2442516903961865e+02;
cy_d = 2.3584766381177013e+02;

%% Right Camera
disp('Getting Right camera points...')
depth = double(imread('Right_camera_Depthitemsfinal.png'));

[rows,cols] = size(depth);
pointNum = rows * cols;
point2R = zeros(3, pointNum);
colToRemove = [];
k=1;
for row=1:rows
    for col=1:cols
        point = [0 0 0];
        point(1) = (row - cx_d) * (depth(row,col) / fx_d);
        point(2) = (col - cy_d) * (depth(row,col) / fy_d);
        point(3) = depth(row,col);
        if point(3) ~= 0
            point = (R * point') + T;
            point2R(:, k) = point;
        else
            colToRemove = [colToRemove c];
        end
        k=k+1;
    end
end
point2R(:,colToRemove)=[];
%%
%Use intrinsic parameters from calibration
FRight = [ 571.235895637785570 ; 563.320611929919780 ];
CRight = [ 327.8227  ; 243.302264022088140 ];
%[ 327.8227 ; 227.2385 ];266.460149510435430
[r2, c2] = size(point2R);

coor2d = zeros(2, c2);
for c=1:c2
    coor2d(1,c) = round(((FRight(1) * point2R(1,c)) / point2R(3,c)) + CRight(1));
    coor2d(2,c) = round(((FRight(2) * point2R(2,c)) / point2R(3,c)) + CRight(2));
end 
%%
colorR = imread('Right_camera_RBGitemsfinal.png');

img2R = [];
img2G = [];
img2B = [];
colToRemove2 = [];

for c=1:c2
    row = coor2d(1,c);
    col = coor2d(2,c);
    if ((row > 0) && (row < 481))
        if ((col > 0) && (col < 641))
            img2R = [img2R colorR(row, col,1)];
            img2G = [img2G colorR(row, col,2)];
            img2B = [img2B colorR(row, col,3)];
        else
            colToRemove2 = [colToRemove2 c];
        end
    else
        colToRemove2 = [colToRemove2 c];
    end
end 
%%
colorR = [img2R; img2G; img2B];
point2R(:,colToRemove2)=[];
%ptCloud = pointCloud();
%%
[r, c] = size(point2R);

%pcshow(point2R(:,1:c)', colorR(:,1:c)');
%view(-90,90);
X = point2R(1,:)';
Y = point2R(2,:)';
Z = point2R(3,:)';
scatter3(Y,X,-Z,5,im2double(colorR(:,:))')
% 
% % scatter3(point2L(1,1:10:c)',point2L(3,1:10:c)', -point2L(2,1:10:c)')
% view(0,90)
disp('Done   ')

%% Left Camera
disp('Getting left camera points...')
depth = double(imread('Left_camera_Depthitemsfinal.png'));

[rows,cols] = size(depth);
coor3d = zeros(rows,cols,3);
pointNum = rows * cols;
point2L = zeros(3, pointNum);
colToRemove = [];
k=1;
for row=1:rows
    for col=1:cols
        point = [0 0 0];
        point(1) = (row - cx_d) * (depth(row,col) / fx_d);
        point(2) = (col - cy_d) * (depth(row,col) / fy_d);
        point(3) = depth(row,col);
        if point(3) ~= 0
            point = (R * point') + T;
            point2L(:, k) = point;
        else
            colToRemove = [colToRemove c];
        end
        k=k+1;
    end
end
point2L(:,colToRemove)=[];
%%
%Use intrinsic parameters from calibration
FLeft = [ 551.0444 ; 551.4409 ]; 
CLeft = [ 327.8227 ; 227.2385 ];
[r2, c2] = size(point2L);
coor2d = zeros(2, c2);
for c=1:c2
    coor2d(1,c) = round(((FLeft(1) * point2L(1,c)) / point2L(3,c)) + CLeft(1));
    coor2d(2,c) = round(((FLeft(2) * point2L(2,c)) / point2L(3,c)) + CLeft(2));
end 
%%
colorL = imread('Left_camera_RBGitemsfinal.png');
img2R = [];
img2G = [];
img2B = [];
colToRemove2 = [];
for c=1:c2
    row = coor2d(1,c);
    col = coor2d(2,c);
    if ((row > 0) && (row < 481))
        if ((col > 0) && (col < 641))
            img2R = [img2R colorL(row, col,1)];
            img2G = [img2G colorL(row, col,2)];
            img2B = [img2B colorL(row, col,3)];
        else
            colToRemove2 = [colToRemove2 c];
        end
    else
        colToRemove2 = [colToRemove2 c];
    end
end 
colorL = [img2R; img2G; img2B];
point2L(:,colToRemove2)=[];
%ptCloud = pointCloud();
%%
[r, c] = size(point2L);
%pcshow(point2L(:,1:c)', colorL(:,1:c)');
%view(-90,90);
X = point2L(1,:)';
Y = point2L(2,:)';
Z = point2L(3,:)';
scatter3(Y,X,-Z,5,im2double(colorL(:,:))')

% % scatter3(point2L(1,1:10:c)',point2L(3,1:10:c)', -point2L(2,1:10:c)')
% view(0,90)
disp('Done   ')

%%
%Together
disp('Combining Points')
Rrl = load('Calib_Results_stereo.mat', 'R');
Trl = load('Calib_Results_stereo.mat', 'T');
om = load('Calib_Results_stereo.mat','om');
om = om.om;
R2 = rotationVectorToMatrix(om);
[r3, c3] = size(point2R);
RR = Rrl.R;
TT = Trl.T;
OtherR = inv(Rrl.R);
Points = zeros(r3, c3);
for c=1:c3
    Points(:, c) = (Rrl.R * point2R(:, c)) - 0;
end
%%
% X = [Points(1,1:5:end) point2L(1,1:5:end)]';
% Y = [Points(2,1:5:end) point2L(2,1:5:end)]';
% Z = [Points(3,1:5:end) point2L(3,1:5:end)]';
Points = [Points point2L];
size(Points)
Colors = [colorR colorL];
size(Colors)

%%
%scatter3(Y,X,-Z,5,im2double(Colors(:,1:5:end))')
pcshow(Points(:,1:10:end)', Colors(:,1:10:end)');

