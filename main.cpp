/* Copyright
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

/* Coautores
   Marco Ramirez
   Juan Carlos Aguilera Perez
   Aurelio Puebla
   Fernando Lopez
*/

/* Aspectos de revision
 * Confirmar se mantega dentro del carril
 * Revisar Caidas
 * Agregar que se detenga a los 3 mins
 * Crear rutina de apagado y encendido
*/

/* Reglas
 * Distancia total 5m
 * Franja inicio 18 cm
 * Franja final 30 cm y de color rojo
 * Max tiempo fuera de carril = 3 segs
 * Se considera fuera de carril con 1 pie fuera de la linea
 * Si algo sucede el robot tiene que poder responder autonomamente (levantar)
*/

#include <iostream>
#include "NaoVision.h"
#include "NaoMovement.h"

#include <alproxies/allandmarkdetectionproxy.h>
#include <alproxies/altexttospeechproxy.h>
#include <alproxies/almemoryproxy.h>


using namespace std;
using namespace AL;
using namespace cv;

string getNaoMark(AL::ALLandMarkDetectionProxy landMark, AL::ALVideoDeviceProxy camProxy, AL::ALMemoryProxy memory, const string clientName ,  string parameterClientName);


int main(int argc, char *argv[]) {
    const int port = 9559;
    string ip = argv[1];        // NAO ip
    cout << "IP: " << ip << endl;

    AL::ALMemoryProxy memory(ip,port);
    AL::ALLandMarkDetectionProxy landMark(ip,port);
    AL::ALTextToSpeechProxy textToSpeech(ip,port);
    AL::ALMotionProxy motion(ip,port);
    AL::ALVideoDeviceProxy camProxy(ip,port);
    AL::ALRobotPostureProxy posture(ip,port);

    bool LOCAL = false;         // Flap for the kind of execution (local or remote).
    bool NAO = true;
    char key = 'x';
    double angleToBlackLine;    // Angle of the detected line.
    //-------Filter Variables----------
    int blackArea = 0;
    int yellowArea = 0;
    int brownArea = 0;
    int whiteArea = 0;
    int redArea = 0;
    float finalArea;
    //-------Filter Variables----------
    //------NaoMark Variables---------
    string expectedMarkID = "";
    string parameterClientName = "Cam_Test_LandMark";
    int period = 500;
    landMark.subscribe("Test_LandMark",period, 0.0);
   // alMemory.subscribeToEvent("MiddleTactilTouch");
    const string clientName = camProxy.subscribe("Cam_Test_LandMark", AL::kQVGA, AL::kBGRColorSpace, 30);
    int cont = 0;
    string naoMark;
    //------NaoMark Variables---------
    int flagGoalIsNear = 0;
    int counterR = 0;
    int counterW = 0;
    int counterB = 0;
    Mat src,src2;
    NaoVision naoVision(ip, port, LOCAL);
    NaoMovement naoMovement(ip, port, LOCAL);
    VideoCapture cap(1);        // Class for video capturing from video files or cameras.

    bool onNear = false;
    bool finish = false;

    //variables boton
    float sale = 0.0;
    bool Nxt = false;

    naoVision.visualCompass();
    while (key != 27 && !finish){
        if (NAO) {
            src = naoVision.getImageFrom(NaoVision::BOTTOM_CAMERA);
        } else {
            cap >> src;
            naoVision.setSourceMat(src);
        }

        //naoVision.calibrateColorDetection();
        //naoVision.colorFilter(src);
       // brownArea = naoVision.getAreaBrownColor(src);
        whiteArea = naoVision.getAreaWhiteColor(src);
       // redArea = naoVision.getAreaRedColor(src);
        //finalArea = naoVision.FinalLineFilterRelayRace(src); //Agregar al proyecto!

        if(redArea >= 25){
            cout<<"Metal " << redArea << endl;
            counterR++;
            if(counterR >= 10){
                expectedMarkID = "64]";
                textToSpeech.say("Metal");
                break;
            }
        }
        if(whiteArea >= 15){
            cout<<"Plastico " << whiteArea << endl;
            //expectedMarkID = "80]";REAL
            counterW++;
            if(counterW >= 10){
                expectedMarkID = "119";
                textToSpeech.say("Plastico");
                break;
            }
        }
        if(brownArea >= 25){
            cout<<"Carton " << brownArea << endl;
            //expectedMarkID = "108";REAL
            counterB++;
            if(counterB >= 10){
                expectedMarkID = "114";
                textToSpeech.say("Carton");
                break;
            }

        }
        key = waitKey(10);
        for (int i = 0; i < 250000; i++);   // Delay.
    }



    while(!Nxt)
    {

   // cout<<"Espera boton"<<endl;
    sale = memory.getData("MiddleTactilTouched");
        if (sale > 0.5f)
        {
            Nxt = true;

     //       cout<<"boton presionado"<<endl;
        }
    }

//Posicion para buscar naoMarks
    posture.goToPosture("Crouch",0.5);
    motion.angleInterpolation("HeadYaw",0.0, 1.0 ,true);//voltear directamente al frente
    usleep(1000000);

    cout<<"Detectando NaoMark"<<endl;

    naoMark = getNaoMark(landMark,camProxy,memory,clientName,parameterClientName);
    cout << naoMark << endl;


    //Buscar naoMark Derecha
    if(naoMark == expectedMarkID){
        //naoMark = getNaoMark(landMark,camProxy,memory,clientName,parameterClientName);
        cout << naoMark << endl;
        cout << "WalkFront" << endl;
        cont = 1;
    }else{
        //Girar Derecha
        motion.angleInterpolation("HeadYaw",-M_PI/5, 1.0 ,true);
        usleep(1000000);
        naoMark = getNaoMark(landMark,camProxy,memory,clientName,parameterClientName);
        cout << naoMark << endl;
        if(naoMark == expectedMarkID){
            cout << "WalkRight" << endl;
        }else{
            //Girar Izquierda
            motion.angleInterpolation("HeadYaw",M_PI/5, 1.0 ,true);
            usleep(1000000);
            naoMark = getNaoMark(landMark,camProxy,memory,clientName,parameterClientName);
            cout << naoMark << endl;
            if(naoMark == expectedMarkID){
             cout << "Walk left" << endl;
            }
            else{
                cout << "NaoMark Not detected look again for it " << endl;
            }
        }
        //Regresar la cabeza a la posicion central para el caminado.
        motion.angleInterpolation("HeadYaw",0.0, 1.0 ,true);
        usleep(1000000);

    }

    cout << cont << endl;
    //..,,,,,,,,,,,Stop sequence,,,,,,,,,,,,,
    posture.goToPosture("Crouch",0.5);
    motion.setStiffnesses("Body",0);
    landMark.unsubscribe("Test_LandMark");
    camProxy.unsubscribe("Cam_Test_LandMark");
    //alMemory.subscribeToEvent("MiddleTactilTouch");
    //..,,,,,,,,,,,Stop sequence,,,,,,,,,,,,,

    //Guardar todo esto en una funcion
    naoVision.unsubscribe();
    naoMovement.stop();

    return 0;
}


string getNaoMark(AL::ALLandMarkDetectionProxy landMark, AL::ALVideoDeviceProxy camProxy, AL::ALMemoryProxy memory, const string clientName ,  string parameterClientName ){

    int contador;
    bool detected = false;
    AL::ALValue markInfo = "";

    Mat src;
    //string parameterClientName = "Test_LandMark";
    //int period = 500;
    //landMark.subscribe("Test_LandMark",period, 0.0);
    //const std::string clientName = camProxy.subscribe("Test_LandMark", AL::kQVGA, AL::kBGRColorSpace, 30);

    string markID = "";

    do{
    camProxy.setActiveCamera(0); //conect to bottom camera
    camProxy.setResolution(parameterClientName, AL::kQVGA);
    /** Create an cv::Mat header to wrap into an opencv image.*/
    Mat imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);
    /** Create a OpenCV window to display the images. */
    //cv::namedWindow("images");

    ALValue img = camProxy.getImageRemote(clientName);

    /** Access the image buffer (6th field) and assign it to the opencv image
    * container. */
    imgHeader.data = (uchar*) img[6].GetBinary();
    /** Tells to ALVideoDevice that it can give back the image buffer to the
    * driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
    camProxy.releaseImage(clientName);
    /** Display the iplImage on screen.*/
    src = imgHeader.clone();
    imshow("src", src);

    do{
      //  cout<<"entra al while2"<<endl;
        markInfo = memory.getData("LandmarkDetected",0);
    //    cout<<"markInfo "<<markInfo.getSize()<<endl;
    }while(markInfo.getSize()<=2);

    // cout<<"Detecta Landmark"<<endl;

        markID = markInfo[1][0][1].toString().substr(1,3);
    // cout<<"MarkID"<<markID<<endl;
    }while(markInfo.getSize() <=2);
    // cout<<"Sale del while"<<endl;
    //Solo es para que compile!!!!!!!!!!
    //Modificar para que regrese si encontro o no la naoMark
    return markID;
}
