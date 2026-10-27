#include <Servo.h>
#include <CheapStepper.h>

// -------- Pins --------
const int irPin = A0;
const int raindropPin = A1;
const int conductivePin = 12;
const int servoPin = 6;

//------------------αρχικοποίηση μεταβλητών μετρήσεων----------------

int conductivePinValue = 0;
int raindropValue = 0;
int stableValue = 0; //μεταβλητή που θα αποθηκευτεί η μέση τιμή των μετρήσεων ir

//-----------μεταβλητές ρύθμισης IR-------------
unsigned long lastGroupTime = 0;
const unsigned long groupWait = 2000; //περιμένει 2s μέχρι να πάρει την επόμενη ομάδα μετρήσεων
unsigned long lastSampleTime = 0;
const unsigned long sampleWait = 5; //παίρνει μία μέτρηση ανά 5ms
long sum = 0;
int count = 0;
const int maxNumberOfCounts = 40;

// Stepper pins
const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;

// -------- IR Thresholds --------
const int irThresholdOn = 150; //ενεργοποίηση του IR για ένδειξη τιμής άνω των 150
const int irThresholdMinOpaque = 450; //θεώρηση αδιαφανές αντικειμένου για τιμή άνω των 450

//---------raindrop Thresholds--------
const int raindropThresholdOn = 300;

// --------μετρητές χρόνου --------
const unsigned long SERVO_DELAY = 1000; // 1 sec μετά τον stepper
const unsigned long STEPPER_WAIT = 11000; // 11 sec μετά το τέλος της 1ης κίνησης
const unsigned long SERVO_STEP_TIME = 20; // Ταχύτητα servo (όσο μεγαλύτερο τόσο πιο αργό)

// --------stepper-servo--------
Servo myservo;
CheapStepper stepper(IN1, IN2, IN3, IN4);
int objectType = 0;
//1 -> μέταλλο
//2 -> υγρό
//3 -> διαφανές
//4 -> αδιαφανές

// --------Καταστάσεις--------
enum SystemState { //δήλωση καταστάσεων
  IDLE, //idle 0 (κατάσταση αναμονής)
  WAIT_FOR_SERVO, // wait_for_servo 1
  SERVO_MOVING, // servo moving 2
  WAIT_FOR_STEPPER_RETURN, // wait_for_stepper_return 3
  FULL_LOOP // FULL_LOOP 4 (έχει γίνει ολοκλήρωση όλων των κινήσεων)
};
//SystemState κατάσταση σύστηματος, systemState μεταβλητή κατάστασης λειτουργίας
SystemState systemState = IDLE; //ορισμός αρχικής κατάστασης συστήματος σε αναμονή

// -------- Variables --------
unsigned long stepperTimer = 0; //σημείο εκκίνησης μέτρησης χρόνου για το stepper (πότε ξεκίνησε η αρχική κίνηση)
unsigned long servoTimer = 0; //βοηθάει στην βηματική κίνηση του servo με την SERVO_STEP_TIME
unsigned long servoStartTime = 0; //ο χρόνος (ms) που πρέπει να ξεκινήσει ο servo (StepperTimer + SERVO_DELAY)

int servoPos = 110; //τρέχουσα γωνία servo (90 μοίρες αρχικά - κεντρικά ώστε να μπορεί να κινείται ελέυθερα και προς τις 2 κατευθύνσεις)
//int stepperDir = 0; // +1 ή -1. Αποθηκεύει την αρχική κίνηση του stepper. +1 κινήθηκε +90 (μεταλλικό), -1 κινήθηκε -90 (υγρό)
int servoState = 0; //κλειστός servo πριν από κίνηση
// -------------------------------------------------------------

//βγάζει την τάση από όλα τα πηνία stepper. Μηδενίζεται η κατανάλωση του stepper όταν δεν κινείται για να μην τρέχει/παίρνει ρεύμα ταυτόχρονα με τον servo
void disableStepper() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void setup() {
  pinMode(conductivePin, INPUT);
  pinMode(raindropPin, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  disableStepper(); // όχι ρεύμα στην αρχή στον stepper
  stepper.setRpm(10); //ταχύτητα περιστροφής stepper

  myservo.attach(servoPin); //δήλωση pin servo
  myservo.write(110); //αρχική θέση servo 110 μοίρες
  delay(500);
  myservo.detach(); // όχι ρεύμα στην αρχή στον servo

  Serial.begin(9600);
}

// -------------------------------------------------------------

void loop() {
  //----------------------Μείωση θορύβου σε αισθητήρα IR με μέσο όρο διαδοχικών επαναλήψεων----------------------------

  unsigned long now = millis(); //χρόνος που έχει περάσει από την ενεργοποίηση του arduino
  //Νέα ομάδα μετρήσεων κάθε 2 sec
  if (now - lastGroupTime >= groupWait){
  //ορισμός τελευταίας ομάδας μετρήσεων στην τωρινή στιγμή
    sum = 0;
    count = 0; //μηδενισμός για νέα ομάδα μετρήσεων
  }
  //Δείγμα κάθε 5ms
  if (count < maxNumberOfCounts && now - lastSampleTime >= sampleWait){ //αν οι μετρήσεις είναι λιγότερες από 40 και o χρόνος προηγούμενης μέτρησης φτάσει τα 5ms
    lastSampleTime = now; //ορισμός τελευταίας μέτρησης στην τωρινή στιγμή
    sum += analogRead(irPin); //διαβάζει τιμή από τον IR
    count++;
  }

  //Υπολογισμός μέσου όρου
  if (count == maxNumberOfCounts){ //ο μετρητής φθάνει τις 40 μετρήσεις
    stableValue = sum / maxNumberOfCounts; //μέσος όρος 40 επαναλήψεων
    //Serial.println(stableValue); //Εκτύπωση στο monitor της τιμής
    count++;
  }

  switch (systemState) { //περιπτώσεις συστήματος
  
    // =========================
    case IDLE: //περίπτωση συστήματος σε αναμονή
    // =========================
    if (stableValue >= irThresholdOn) {

      delay(1000);
      raindropValue = analogRead(raindropPin);
      conductivePinValue = digitalRead(conductivePin);

      if (conductivePinValue == LOW){
        objectType = 1; //μέταλλο
        stepper.moveDegrees(true, 90); //περιστροφή stepper 90 μοίρες
      } 
      else if (raindropValue >= raindropThresholdOn){
        objectType = 2; //υγρό
        stepper.moveDegrees(false, 90); //περιστροφή stepper -90 μοίρες
      }
      else if (stableValue >= irThresholdMinOpaque){
        objectType = 4; //αδιαφανές
        stepper.moveDegrees(true, 180); //περιστροφή stepper 180 μοίρες
      }
      else{
        objectType = 3; //διαφανές
        //καμία κίνηση του stepper
      }
      myservo.detach(); // διασφάλιση ότι ο servo είναι απενεργοποιημένος
      stepperTimer = now; //ξεκινάει η μέτρηση του χρόνου από την στιγμή που ο stepper ολοκλήρωσε την πρώτη κίνηση
      servoStartTime = now + SERVO_DELAY; //η μέτρηση του χρόνου έναρξης για το servo ξεκινάει 1 sec μετά το χρονόμετρο του stepper - εξασφαλίζεται ότι έχει τελειώσει η κίνηση του stepper
      systemState = WAIT_FOR_SERVO; // το σύστημα μπαίνει σε κατάσταση αναμονής κίνησης του servo
    }
    break;
      
    // =========================
    case WAIT_FOR_SERVO: //περίπτωση κατάστασης αναμονής για servo
    // =========================
    if (now >= servoStartTime) { //όταν περάσει 1 δευτερόλεπτο
      // απενεργοποίηση stepper
      disableStepper();
      
      // ενεργοποίηση servo
      myservo.attach(servoPin);
      servoTimer = now; //χρονόμετρο servo
      servoPos = 110; //αρχικοποίηση servo στις 90 μοίρες
      myservo.write(servoPos);
      servoState = 0; //κλειστός servo πριν την κίνηση του
      
      systemState = SERVO_MOVING; //το σύστημα μπαίνει σε κατάσταση κίνησης του servo
    }
    break;
      
    // =========================
    case SERVO_MOVING: //περίπτωση κατάστασης κίνησης servo
    // =========================
      if (now - servoTimer >= SERVO_STEP_TIME) { //κάνει ένα βήμα κάθε 20ms
        servoTimer = now; //επανορίζεται το timer για έναρξη του επόμενου βήματος
      // Άνοιγμα
        if (servoState == 0 && servoPos > 0) { //όσο η θέση του servo είναι >0 μοίρες, δηλαδή μέχρι να ανοίξει (90 μοίρες κλειστό, 0 ανοιχτό)
          servoPos--; //μείωση κατά ένα βήμα
          myservo.write(servoPos); //κίνηση servo
          if (servoPos == 0){
            servoState = 1; //ανοιχτός servo
          }
        }

        else if (servoState == 1 && servoPos < 110) { //
          servoPos++;
          myservo.write(servoPos);
          if (servoPos == 110){
            servoState = 2; //κλειστός servo μετά από κίνηση
          }
        }
        else if (servoState == 2){
          // απενεργοποίηση servo
          myservo.detach();
          systemState = WAIT_FOR_STEPPER_RETURN; //μπαίνει σε κατάσταση αναμονής επιστροφής stepper
        }
      }
      break;

    // =========================
    case WAIT_FOR_STEPPER_RETURN: //περίπτωση αναμονήw επιστροφής stepper
    // =========================
      if (now - stepperTimer >= STEPPER_WAIT) { //αν περάσουν 6 δευτερόλεπτα από την πρώτη περιστροφή του stepper
  
        // επιβεβαίωση ότι ο servo είναι απενεργοποιημένος
        myservo.detach();
  
        //ενεργοποίηση stepper
        if (objectType == 1){ //μεταλλικό
          stepper.moveDegrees(false, 90); //επιστροφή στην αρχική θέση με περιστροφή stepper -90 μοίρες
        }
        else if (objectType == 2){ //υγρό
          stepper.moveDegrees(true, 90); //επιστροφή στην αρχική θέση με περιστροφή stepper +90 μοίρες
        }
        else if (objectType == 4){ //αδιαφανές
          stepper.moveDegrees(false, 180); //επιστροφή στην αρχική θέση με περιστρφή stepper -180 μοίρες
        }
        //objectType == 3 -> διαφανές -> καμία περιστροφή stepper
        disableStepper(); //απενεργοποίηση stepper
        
        systemState = FULL_LOOP; //επαναφορά συστήματος σε αδράνεια
      }
      break;
  
      case FULL_LOOP:
        systemState = IDLE;
        servoState = 0;
        break;
  }
}
