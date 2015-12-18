/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <aruco/aruco.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
string TheBoardConfigFile;
bool The3DInfoAvailable = false;
float TheMarkerSize = -1;
VideoCapture TheVideoCapturer;
Mat TheInputImage, TheInputImageCopy;
CameraParameters TheCameraParameters;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;

string TheOutVideoFilePath;
cv::VideoWriter VWriter;

void cvTackBarEvents(int pos, void *);
pair< double, double > AvrgTime(0, 0); // determines the average time required for detection
double ThresParam1, ThresParam2;
int iThresParam1, iThresParam2;
int waitTime = 0;

int dir = 60;
int step = 48;
int reset = 49;

int pasos = 52;
int VELOCIDAD = 750;
int repeticiones = 2;

char high[] = "1";
char low[] = "0";

int testNumber = -1;
    //t1: Sólo detectar el patron en una fotografía.
    //t2: Sólo detectar el patron en un video.
    //t3: Entregar distancia al centro de la imagen en una fotografía.
    //t4: Entregar distancia al centro de la imagen en un video.
    //t5: Indicar instrucción de la dirección del movimiento en lenguaje
    //    natural a travéz de la consola y seguir la instrucción moviendo
    //    de manera manual
    //t6: Movimiento automático, a través del sistema implementado
    //    Debido a los archivos que se debe acceder para enviar instrucciónes
    //    a los pines GPIO, se debe ejecutar con permisos de super usuario


/************************************
 *
 *
 *
 *
 ************************************/

FILE *myOutputHandle = NULL;

int exportPin(int pin){
    //printf("\nStarting GPIO output program\n");
    char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];
    sprintf(GPIOString, "%d", pin);
    sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", pin);
    sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", pin);
 
    // Export the pin
    if ((myOutputHandle = fopen("/sys/class/gpio/export", "ab")) == NULL){
        printf("Unable to export GPIO pin\n");
        return 1;
    }
    strcpy(setValue, GPIOString);
    fwrite(&setValue, sizeof(char), 2, myOutputHandle);
    fclose(myOutputHandle);
 
    // Set direction of the pin to an output
    if ((myOutputHandle = fopen(GPIODirection, "rb+")) == NULL){
        printf("Unable to open direction handle\n");
        return 1;
    }
    strcpy(setValue,"out");
    fwrite(&setValue, sizeof(char), 3, myOutputHandle);
    fclose(myOutputHandle);
    return 0;
}

int unexportPin(int pin){
// Unexport the pin
    char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];
    sprintf(GPIOString, "%d", pin);
    sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", pin);
    sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", pin);

    if ((myOutputHandle = fopen("/sys/class/gpio/unexport", "ab")) == NULL) {
        printf("Unable to unexport GPIO pin\n");
        return 1;
    }
    strcpy(setValue, GPIOString);
    fwrite(&setValue, sizeof(char), 2, myOutputHandle);
    fclose(myOutputHandle);
    //printf("\nCompleted GPIO output program\n");
    return 0;
}

int writeGPIO(int GPIONumber, char* data){
    int GPIOPin=GPIONumber;
 
    //printf("\nStarting GPIO output program\n");
    FILE *myOutputHandle = NULL;
    char setValue[4], GPIOString[4], GPIOValue[64], GPIODirection[64];
    sprintf(GPIOString, "%d", GPIOPin);
    sprintf(GPIOValue, "/sys/class/gpio/gpio%d/value", GPIOPin);
    sprintf(GPIODirection, "/sys/class/gpio/gpio%d/direction", GPIOPin);
 
    // Set output to high
    if ((myOutputHandle = fopen(GPIOValue, "rb+")) == NULL){
        printf("Unable to open value handle\n");
        return 1;
    }
    strcpy(setValue, data); // Set value high
    fwrite(&setValue, sizeof(char), 1, myOutputHandle);
    fclose(myOutputHandle);
    return 0;
}

bool readArguments(int argc, char **argv) {

    if (argc < 3) {
        cerr << "Invalid number of arguments" << endl;
        cerr << "Usage: (in.avi|live) boardConfig.yml [intrinsics.yml] [size] [out] [t#]" << endl;
        return false;
    }
    TheInputVideo = argv[1];
    TheBoardConfigFile = argv[2];
    if (argc >= 4)
        TheIntrinsicFile = argv[3];
    if (argc >= 5)
        TheMarkerSize = atof(argv[4]);
    if (argc >= 6)
        TheOutVideoFilePath = argv[5];
    if (argc >= 7)
        {
            char c  = argv[6][1];
            testNumber = c - '0';
        }


    if (argc == 4)
        cerr << "NOTE: You need makersize to see 3d info!!!!" << endl;

    return true;
}

std::string FloatToString (float number){
    std::ostringstream buff;
    buff<<number;
    return buff.str();   
}

void addLocationData(){
    // Información de Posición
    putText(TheInputImageCopy, "X: " + FloatToString(TheBoardDetector.getDetectedBoard().Tvec.at<float>(0,0)), Point(10,20), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    putText(TheInputImageCopy, "Y: " + FloatToString(TheBoardDetector.getDetectedBoard().Tvec.at<float>(1,0)), Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    putText(TheInputImageCopy, "Z: " + FloatToString(TheBoardDetector.getDetectedBoard().Tvec.at<float>(2,0)), Point(10,40), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);

    // Información de Rotación
    putText(TheInputImageCopy, "Rx: " + FloatToString(TheBoardDetector.getDetectedBoard().Rvec.at<float>(0,0)), Point(10,70), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    putText(TheInputImageCopy, "Ry: " + FloatToString(TheBoardDetector.getDetectedBoard().Rvec.at<float>(1,0)), Point(10,80), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    putText(TheInputImageCopy, "Rz: " + FloatToString(TheBoardDetector.getDetectedBoard().Rvec.at<float>(2,0)), Point(10,90), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
}

int printInstruction(){
    cout << "===========" << endl;
    string horizontalInstruction;
    float xPos = TheBoardDetector.getDetectedBoard().Tvec.at<float>(0,0);
    float yPos = TheBoardDetector.getDetectedBoard().Tvec.at<float>(1,0);
    float zPos = TheBoardDetector.getDetectedBoard().Tvec.at<float>(2,0);
    
    int acercarHorizontal = 0;
    int acercarVertical = 0;
    int posicionHorizontal = 0;
    int posicionVertical = 0;
    int posicionCercania = 0;
    
    if(zPos <= 40){
        cout << "Lo suficientemente cerca" << endl;
        posicionCercania = -1;
    }
    else{
        if(xPos < -5 || xPos > 5){
            acercarHorizontal = -1;
        }
        if(yPos < -5 || yPos > 5){
            acercarVertical = -1;
        }
        if(acercarHorizontal && acercarVertical ){
            cout << "Acercar al Marcador" << endl;
        }
    }
    
    if (xPos < -1.5)
        horizontalInstruction = "Izquierda";
    else if (xPos > 1.5)
        horizontalInstruction = "Derecha";
    else
    {
        horizontalInstruction = "Horizontal OK";
        posicionHorizontal = -1;
    }
    cout << horizontalInstruction << endl;
    putText(TheInputImageCopy, "X: " + FloatToString(xPos), Point(10,20), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    putText(TheInputImageCopy, horizontalInstruction, Point(10,30), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    

    string verticalInstruction;
    if (yPos < -1.5)
        verticalInstruction = "Arriba";
    else if (yPos > 1.5)
        verticalInstruction = "Abajo";
    else
    {
        verticalInstruction = "Vertical OK";
        posicionVertical = -1;
    }
    cout << verticalInstruction << endl;
    putText(TheInputImageCopy, "Y: " + FloatToString(yPos), Point(10,50), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    putText(TheInputImageCopy, verticalInstruction, Point(10,60), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    
    if(posicionVertical && posicionHorizontal && posicionCercania)
        return -1;
    else
        return 0;
}



int move(){
    char high[] = "1";
    char low[] = "0";
    float xPos = TheBoardDetector.getDetectedBoard().Tvec.at<float>(0,0);
    int centered = 1;
    
    printf("=========%f==========\n", xPos);
    
    if (xPos < -1.5)
    {
        writeGPIO(dir, low);
        centered = 0;
        xPos = xPos * -1;
    }
    else if (xPos > 1.5)
    {
        writeGPIO(dir, high);
        centered = 0;
    }
    
    if (centered)
    {
        printf("Inexportando pines...\n");
        unexportPin(dir);
        unexportPin(step);
        unexportPin(reset);
        return -1;
    }
    else
    {
        printf("Girando...\n");
        int pasos2 = pasos * xPos;
        for (int i = 0; i<pasos2; i++)
        {
            writeGPIO(step, high);
            writeGPIO(step, low);
            usleep(VELOCIDAD);
        } 
    }

    return 0;
}

/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc, char **argv) {
    try {
        if (readArguments(argc, argv) == false)
            return 0;
        // parse arguments
        TheBoardConfig.readFromFile(TheBoardConfigFile);
        // read from camera or from  file
        if (TheInputVideo == "live") {
            TheVideoCapturer.open(0);
            waitTime = 10;
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_WIDTH,640);
            TheVideoCapturer.set(CV_CAP_PROP_FRAME_HEIGHT,240);
        } else
            TheVideoCapturer.open(TheInputVideo);
        // check video is open
        if (!TheVideoCapturer.isOpened()) {
            cerr << "Could not open video" << endl;
            return -1;
        }

        if(testNumber == 6){
            printf("Exportando pines...\n");
            exportPin(dir);
            exportPin(step);
            exportPin(reset);


            printf("Reseteando...\n");
            writeGPIO(reset, low);
            usleep(100000);
            writeGPIO(reset, high);
            writeGPIO(dir, high);
        }

        // read first image to get the dimensions
        TheVideoCapturer >> TheInputImage;

        // Open outputvideo
        if (TheOutVideoFilePath != "")
            VWriter.open(TheOutVideoFilePath, CV_FOURCC('M', 'J', 'P', 'G'), 15, TheInputImage.size());

        // read camera parameters if passed
        if (TheIntrinsicFile != "") {
            TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
            TheCameraParameters.resize(TheInputImage.size());
        }

        TheBoardDetector.setParams(TheBoardConfig, TheCameraParameters, TheMarkerSize);
        TheBoardDetector.getMarkerDetector().getThresholdParams(ThresParam1, ThresParam2);
        TheBoardDetector.getMarkerDetector().setCornerRefinementMethod(MarkerDetector::HARRIS);
        TheBoardDetector.set_repj_err_thres(1.5);

        iThresParam1 = 13;//ThresParam1;
        iThresParam2 = 13;//ThresParam2;

        int index = 0;
        // capture until end of the video
        cout << "Running..." << endl;
        do {
        
            if(TheInputVideo == "live")
            {
                for(int buffer = 0; buffer <= 5 ; buffer++)
                    TheVideoCapturer.grab();
            }
        
            TheVideoCapturer.retrieve(TheInputImage);
            TheInputImage.copyTo(TheInputImageCopy);
            index++; // number of images captured
            // Detection of the board
            float probDetect = TheBoardDetector.detect(TheInputImage);
            // print marker borders
            if(testNumber != -1)
            {
                for (unsigned int i = 0; i < TheBoardDetector.getDetectedMarkers().size(); i++)
                    TheBoardDetector.getDetectedMarkers()[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 1);
            }

            // print board
            if (TheCameraParameters.isValid()) {
                if (probDetect > 0.2) {
                    // board detected
                    if(testNumber != -1){
                        // Se dibujan los ejes desde el centro del tablero
                        CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheBoardDetector.getDetectedBoard(), TheCameraParameters);
                        //Pruebas
                        if (testNumber == 3 || testNumber == 4){
                            addLocationData(); // Se agrega información de la posición a la imagen de salida
                        }
                        else if (testNumber == 5){
                            // Se imprime por consola, y se agrega a la imagen de salida
                            if(printInstruction())
                                return 0;
                        }
                        else if (testNumber == 6){
                            // Seguimiento Automático
                            if(move())
                            {
                                if (TheOutVideoFilePath != "") {
                                    cv::Mat roi = TheInputImageCopy(cv::Rect(0, 0, TheInputImageCopy.cols / 3, TheInputImageCopy.rows / 3));
                                    VWriter << TheInputImageCopy;
                                    VWriter << TheInputImageCopy;
                                    VWriter << TheInputImageCopy;
                                    VWriter << TheInputImageCopy;
                                }
                                return 0;
                            }
                        }

                    }
                }
                else{
                    cout << "Tablero no Detectado" << endl;
                }
            }
            // DONE! Easy, right?

            // write to video if required
            if (TheOutVideoFilePath != "") {
                cv::Mat roi = TheInputImageCopy(cv::Rect(0, 0, TheInputImageCopy.cols / 3, TheInputImageCopy.rows / 3));
                VWriter << TheInputImageCopy;
            }




            cv::waitKey(waitTime); // wait for key to be pressed
            
        } while (TheVideoCapturer.grab());


    } catch (std::exception &ex)

    {
        cout << "Exception :" << ex.what() << endl;
    }
}
