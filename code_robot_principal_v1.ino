// Include libraries:
#include <AccelStepper.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string.h>

//definition des pins
#define moteurDdirPin 5
#define moteurDstepPin 2   //axe x
#define moteurGdirPin 7    //axe z
#define moteurGstepPin 4
#define motorInterfaceType 1

//definition des moteurs
AccelStepper moteurG = AccelStepper(motorInterfaceType, moteurGstepPin, moteurGdirPin);
AccelStepper moteurD = AccelStepper(motorInterfaceType, moteurDstepPin, moteurDdirPin);

//position relative du robot, x, y en metre et angle rad
double X ;
double Y;
double A;

double VmaxD = 1600; // vitesse des roues max
double VmaxG = 1600/0.9057;



int etat; //commande de position par le rasp;   etat: 0: stop, 1: rotation, 2: translation
double x, y; // position en mm
float ordre; // l'ordre envoyé
int alpha; // entre 0 et 360
float res[3] = {}; // tableau contenant les paramètres géométriques

double facteurMoteurG = 0.00005*1.0267*1.39;  // facteur metre/step
double facteurMoteurD = 0.00005*1.0267*1.39;  // si deplacement de 9cm au lieu de 10cm => facteur * 0.9
double facteurRotation = 0.1358;      // si 90 deg au lieu de 180 => facteur *2
double largeur = 0.27; // largeur du robot en m

//initialisation node sender
ros::NodeHandle  nh;
std_msgs::String msg_send;
ros::Publisher chatter("feedback_move", &msg_send);

void messageCb( const std_msgs::String& toggle_msg){
  String msg = toggle_msg.data;
  char* data = toggle_msg.data;
  msg_send.data = data;
  chatter.publish( &msg_send );
  nh.spinOnce();
  int* ordre = split(msg);
  int etat = ordre[0];
  int x = ordre[1];
  int y = ordre[2];
  int alpha = ordre[3];

    if (etat == 0) {
      stopMoteurs();
    }

    if (etat == 1) { //rotation: recoit alpha;
      rotation(alpha);
      msg_send.data = "success";
      chatter.publish( &msg_send );
      nh.spinOnce();
    }


    else if (etat == 2) { //translation: recoit x,y;

      boolean translationComplete = translation(x, y);  //true si pas de message dans le serialPort, false si l'arduino reçoit un message


      if (translationComplete == true) {
        msg_send.data = "success";
        chatter.publish( &msg_send );
        nh.spinOnce();
      }
    }
}

//initialisation node receiver
ros::Subscriber<std_msgs::String> sub("order_move", &messageCb );

void setup() {
  Serial.begin(9600);
  
  moteurG.setMaxSpeed(VmaxG);
  moteurG.setAcceleration(300*2);
  moteurD.setMaxSpeed(VmaxD);
  moteurD.setAcceleration(300*2);

  //initialisation des nodes
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);


}

void loop() {
  nh.spinOnce();
  delay(1);
  }


  boolean translation(double x, double y) {
    x = x/1000;
    y = y/1000;
    char* data = "Fonction translation appelee";
    msg_send.data = data;
    chatter.publish( &msg_send );
    nh.spinOnce();
    float Lg;
    float Ld;
    geometrie(x, y);
    float Lpetit = res[0];
    float Lgrand = res[1];
    float sgn = res[2];

    if (sgn > 0) { // fixer les vitesses en sachant savoir quel roue est la grande longueur et quelle roue est la petite longueur: ici, c'est la gauche qui devra aller vite
      Lg = Lgrand;
      Ld = Lpetit;
      moteurG.setAcceleration(300*2);
      moteurG.setMaxSpeed(VmaxG); // pb: si un moteur finit avant l'autre (adapter accéleration?)
      moteurD.setAcceleration(300*2 * Lpetit / Lgrand);
      moteurD.setMaxSpeed(VmaxD * Lpetit / Lgrand);
    }
    if (sgn < 0) {
    // ici c la roue droite qui devra aller plus vite
      Lg = Lpetit;
      Ld = Lgrand;
      moteurG.setMaxSpeed(VmaxG * Lpetit / Lgrand); // pb: si un moteur finit avant l'autre (adapter accéleration?)
      moteurG.setAcceleration(300*2 * Lpetit / Lgrand);
      moteurG.setAcceleration(300*2 );
      moteurD.setMaxSpeed(VmaxD);
    }

    moteurG.move(long(-Lg / facteurMoteurG)); // on donne les consignes de mouvement aux moteurs
    moteurD.move(long(Ld / facteurMoteurD));
    
    while ((moteurG.isRunning() || moteurD.isRunning()) ) {// tant que les 2 moteurs ont pas fini et qu'on a pas recu de nouvelle donnée
      moteurG.run(); // les moteurs effectuent leurs ordres
      moteurD.run();

    }

    return true;
  }

  void rotation ( double angle0) {            //rotation attention pas de stoppe pendant la rotation
    angle0 = angle0 * PI / 180; // on convertit l'angle
    if (angle0 > PI) {
      angle0 -= 2 * PI;
    }
    moteurG.move(long(angle0 * facteurRotation / facteurMoteurG)); //ajoute nouvelle objectif
    moteurD.move(long(angle0 * facteurRotation / facteurMoteurD)); //tourne dans le sens inverse
    while (moteurG.isRunning() == true | moteurD.isRunning() == true) { // question: remplacer le isrunning?
      moteurG.run();
      moteurD.run();

    }

  }


  void stopMoteurs() {
    moteurG.stop();   //nouvel objectif pour s'arreter le plus rapidement possible
    moteurD.stop();
    while (moteurG.isRunning() == true | moteurD.isRunning() == true) {
      moteurG.run();
      moteurD.run();
    }
  }

  void geometrie(float x, float y) {
    if (y < 0.001) { // si c n'est pas un cercle (mettre valeur limite?)
      res[0] = x;
      res[1] = x; //  mais une ligne droite: les deux roues avancent pareil
      res[2] = 1; // on ne changera pas le réglage de la vitesse
    }

    else { // sinon: on calcule de combien doit avancer chaque roue
      double theta = atan (x / y);
      float R = sqrt((x / 2) * (x / 2) + (y / 2) * (y / 2) ) / cos(theta);
      float Lpetit = 2*(R - largeur / 2) * (PI / 2 - theta);
      float Lgrand = 2*(R + largeur / 2) * (PI / 2 - theta);

      res[0] = Lpetit;
      res[1] = Lgrand;
      res[2] = abs(y) / y; // disjonction des cas rotation à droite ou à gauche
    }
  }

int* split(String input) {
  int first, second, third, fourth;
  static int myList[4];
  
  // Find commas
  int firstComma = input.indexOf(',');
  int secondComma = input.indexOf(',', firstComma + 1);
  int thirdComma = input.indexOf(',', secondComma + 1);
  
  // Get integers
  first = input.substring(0, firstComma).toInt();
  second = input.substring(firstComma + 1, secondComma).toInt();
  third = input.substring(secondComma + 1, thirdComma).toInt();
  fourth = input.substring(thirdComma + 1).toInt();

  String s = input.substring(0, firstComma);
  char* char_array = new char[s.length() + 1];
  char_array[s.length()] = '\0';  
  for (int i = 0; i < s.length(); i++) {
    char_array[i] = s[i];
  }
  msg_send.data = char_array;
  chatter.publish( &msg_send );
  nh.spinOnce();
  s = input.substring(firstComma + 1, secondComma);
  char_array = new char[s.length() + 1];
  char_array[s.length()] = '\0';  
  for (int i = 0; i < s.length(); i++) {
    char_array[i] = s[i];
  }
  msg_send.data = char_array;
  chatter.publish( &msg_send );
  nh.spinOnce();
  s = input.substring(secondComma + 1, thirdComma);
  char_array = new char[s.length() + 1];
  char_array[s.length()] = '\0';  
  for (int i = 0; i < s.length(); i++) {
    char_array[i] = s[i];
  }
  msg_send.data = char_array;
  chatter.publish( &msg_send );
  nh.spinOnce();
  s = input.substring(thirdComma + 1);
  char_array = new char[s.length() + 1];
  char_array[s.length()] = '\0';  
  for (int i = 0; i < s.length(); i++) {
    char_array[i] = s[i];
  }
  msg_send.data = char_array;
  chatter.publish( &msg_send );
  nh.spinOnce();
  
  myList[0] = first;
  myList[1] = second;
  myList[2] = third;
  myList[3] = fourth;

  return myList;
}
