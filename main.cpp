#include <algorithm> // for std::sort
#include <chrono>
#include <dirent.h>
#include <exception>
#include <locale>
#include <opencv2/opencv.hpp>
#include <softPwm.h>
#include <stdio.h>
#include <vector> // for std::vector
#include <wiringPi.h>
#include <wiringSerial.h>

#define PWM_PIN 25 // wiringpi pin (not BCM)

using namespace std;
using namespace cv;

float g_kp, g_ki, g_kd;
float g_offset, g_error, g_error_last;
Mat g_frame;
vector<Mat> g_templates;

Scalar upper_pink = Scalar(130, 50, 50);
Scalar lower_pink = Scalar(180, 255, 255);
Scalar upper_black = Scalar(0, 0, 0);
Scalar lower_black = Scalar(180, 255, 55);

int robot;

#define BASE_SPEED 20;

enum State { LINE_FOLLOWING, IMG_RECOG, PERFORM_TASK, IDLE };
enum State state;

enum CameraPos { UP, DOWN };
enum CameraPos cameraPos;

typedef enum {
  COUNT_SHAPES1,
  COUNT_SHAPES2,
  COUNT_SHAPES3,
  BLUE,
  GREEN,
  RED,
  YELLOW,
  PLAY_MUSIC,
  ALARM_FLASH,
  APPROACH_AND_STOP,
  KICK_BALL,
  TRAFFIC_LIGHT,
  NONE
} Task;

void loadTemplates() {
  // Load all PNG images under the ./templates/ directory
  // Grayscale images

  string directory = "./templates/";
  DIR *dir;
  struct dirent *ent;
  vector<string> filenames; // Store the file names here

  if ((dir = opendir(directory.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      string filename = ent->d_name;
      if (filename.length() >= 4 &&
          filename.substr(filename.length() - 4) == ".png") {
        filenames.push_back(filename); // Store the file names
      }
    }
    closedir(dir);

    // Sort the file names
    sort(filenames.begin(), filenames.end());

    // Load and process the images in sorted order
    for (const auto &filename : filenames) {
      cv::Mat image = cv::imread(directory + filename, cv::IMREAD_GRAYSCALE);
      if (!image.empty()) {
        resize(image, image, Size(320, 240));
        g_templates.push_back(image);
        cout << "Successfully loaded: " << filename << endl;
      }
    }
  } else {
    cerr << "Error opening directory" << endl;
  }
}

void moveCamera() {
  if (cameraPos == DOWN)
    softPwmWrite(PWM_PIN, 25);
  else if (cameraPos == UP)
    softPwmWrite(PWM_PIN, 15);
  delay(500);
  // turn off the servo motor so it doesn't shake
  softPwmWrite(PWM_PIN, 0);
}

void setup(void) {
  wiringPiSetup();
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  softPwmCreate(PWM_PIN, 0, 200);
  state = LINE_FOLLOWING;
  cameraPos = DOWN;
  moveCamera();

  loadTemplates();
}

// Function to detect contours
// returns true if contours are found, false otherwise
// stores the largest contour in largestContour
bool detectContours(const Mat &inputFrame, vector<Point> &largestContour) {
  vector<vector<Point>> contours;
  findContours(inputFrame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    // No contours found, return false
    return false;
  }

  int largest_contour_index = 0;
  double max_area = 0.0;
  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area > max_area) {
      max_area = area;
      largest_contour_index = i;
    }
  }

  largestContour = contours[largest_contour_index];
  return true; // Contours found
}

// Function to perform preprocessing steps
// This function takes a frame of the video as input, performs preprocessing
// steps, and returns a binary mask of the region of interest (black -> white,
// other colors -> black)
void preprocessFrame(const Mat &inputFrame, Mat &outputFrame) {
  // convert input frame to HSV color space
  Mat frame_hsv;
  cvtColor(inputFrame, frame_hsv, COLOR_BGR2HSV);

  // Smooth the image to reduce noise
  int blurSize = 17; // Adjust the size of the blur kernel
  GaussianBlur(frame_hsv, frame_hsv, Size(blurSize, blurSize), 0);

  // Filter out the colors that are not in the range of HSV values of interest
  Mat rangeFilteredMask;

  // method 1: HSV inrange filtering
  inRange(frame_hsv, lower_black, upper_black, rangeFilteredMask);

  // Apply morphological operations to remove small noise and fill in gaps in
  // the mask
  int dilationSize = 12; // Adjust the size of the dilation kernel
  Mat kernel =
      getStructuringElement(MORPH_RECT, Size(dilationSize, dilationSize));
  dilate(rangeFilteredMask, rangeFilteredMask, kernel, Point(-1, -1), 1);
  erode(rangeFilteredMask, outputFrame, kernel, Point(-1, -1), 1);
}

// Function to capture an image from the camera, extract the roi, and return it
Mat imgcap(float yRatio, float heightRatio) {
  Mat frame;
  resize(g_frame, frame, Size(320, 240));

  int x = 0;
  int y = frame.rows * yRatio;
  int width = frame.cols;
  int height = frame.rows * heightRatio;

  // printf("x: %d y: %d width: %d height: %d", x, y, width, height);
  Rect roi(x, y, width, height);

  return frame(roi);
}

// Function to calculate g_error from a given frame
void errorCalc() {
  // read frame
  Mat frame;
  frame = imgcap(0.5, 0.3);

  Mat processed_frame;
  preprocessFrame(frame, processed_frame);

  vector<Point> largestContour;

  Mat resultFrame = frame.clone();

  // if there were contours found
  if (detectContours(processed_frame, largestContour) == true) {
    // find the center of the largest contour and puttext on the frame
    Moments m = moments(largestContour, false);
    Point2f center(m.m10 / m.m00, m.m01 / m.m00);

    // Draw the largest contour and center of the largest contour on the
    // frame
    drawContours(resultFrame, vector<vector<Point>>{largestContour}, -1,
                 Scalar(255, 255, 255), 1);
    circle(resultFrame, center, 5, Scalar(0, 255, 255), -1);

    // draw the distance from the center of the frame to the center of the
    // largest contour
    int distance = (int)center.x - frame.cols / 2;
    g_error = (float)distance;
  } else { // no contours found
    g_error = -0.1;
  }
}

// calculate pid values using g_error and modify offset
void offsetCalc() {
  g_kp = 0.3;
  g_ki = 0.003;
  g_kd = 3.2;

  g_offset = g_kp * g_error + g_ki * g_error + g_kd * (g_error - g_error_last);
}

void sendCmd() {
  int left_speed, right_speed;
  char cmd[50];
  left_speed = BASE_SPEED + g_offset;
  right_speed = BASE_SPEED - g_offset;

  if (left_speed < 0) {
    left_speed *= -1;
    sprintf(cmd, "#Barrff %03d %03d %03d %03d", left_speed, left_speed,
            right_speed, right_speed);
  } else if (right_speed < 0) {
    right_speed *= -1;
    sprintf(cmd, "#Baffrr %03d %03d %03d %03d", left_speed, left_speed,
            right_speed, right_speed);
  } else {
    sprintf(cmd, "#Baffff %03d %03d %03d %03d", left_speed, left_speed,
            right_speed, right_speed);
  }
  serialPuts(robot, cmd);
}

bool existPink() {
  // read frame
  // detect pink color
  Mat frame;
  frame = imgcap(0.5, 0.3);
  inRange(frame, lower_pink, upper_pink, frame);

  imshow("Pink", frame);
  if (waitKey(1) == 'q') {
    return true;
  }

  if (countNonZero(frame) >= 100) { // TODO: adjust threshold
    return true;
  } else {
    printf(" pink pixels: %d ", countNonZero(frame));
    return false;
  }
}

// TODO: finish reset function
void reset() {}

Task templateMatching() {
  Task task = NONE;
  // auto start_time = chrono::high_resolution_clock::now();
  // read frame
  // if (chrono::high_resolution_clock::now() - start_time <
  //     chrono::seconds(10)) {
  //   printf("Timeout\n");
  //   return NONE;
  // }
  Mat frame;
  frame = imgcap(0, 1);
  cvtColor(frame, frame, COLOR_BGR2GRAY);
  equalizeHist(frame, frame);
  // loop through all templates and find the best match
  for (int i = 0; i < g_templates.size(); i++) {
    printf("Template No.%d: ", i);
    Mat result;
    // printf("frame: %d template: %d", frame.size().height,
    //        g_templates[i].size().height);
    matchTemplate(frame, g_templates[i], result, TM_CCOEFF_NORMED);
    double minVal, maxVal;
    Point minLoc, maxLoc;

    imshow("result", result);
    imshow("frame", frame);
    if (waitKey(1) == 'q') {
      return PLAY_MUSIC;
    }

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    printf("maxVal: %f\n", maxVal);
    if (maxVal > 0.6 && maxVal > maxVal) {
      printf("Better match found... \n");
      maxVal = maxVal;
      task = (Task)i;
    }
  }
  return task;
}

// perform the appropriate task according to input
void performTask(Task task) {
  bool taskIncomplete = true;
  while (taskIncomplete) {
    switch (task) {
    case PLAY_MUSIC:
      printf("executing task PLAY_MUSIC");
      // system("omxplayer /home/pi/Desktop/p6/3.MP3");
      taskIncomplete = false;
      break;
    }
  }
}

int main() {
  setup();
  robot = serialOpen("/dev/ttyAMA0", 57600); // returns int, -1 for error

  VideoCapture cap(0);

  getchar();
  state = LINE_FOLLOWING;

  while (1) {
    printf("state: %d\n", (int)state);

    cap >> g_frame;

    Task currentTask = NONE;
    switch (state) {
    case LINE_FOLLOWING:
      if (existPink()) {
        state = IMG_RECOG;
        break;
      }
      // calculate g_error from the center of the line
      errorCalc();
      // calculate offset of motors using pid
      offsetCalc();
      // send command to the car
      sendCmd();
      break;
    case IMG_RECOG:
      // move camera up
      if (cameraPos != UP) {
        cameraPos = UP;
        moveCamera();
      }
      // capture image
      // Mat frame;
      // frame = imgcap( );
      // template matching
      currentTask = templateMatching();
      if (currentTask != NONE) {
        // move camera DOWN
        cameraPos = DOWN;
        moveCamera();

        // perform task
        performTask(currentTask);
        currentTask = NONE;

        state = LINE_FOLLOWING;

        break;
      }
      break;
    case IDLE:
      reset();
      break;
    }
  }
}
