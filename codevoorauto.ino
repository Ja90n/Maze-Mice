#include <Wire.h>
extern "C" { 
#include "utility/twi.h"
}
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ----------------- Motorpinnen -----------------
int motorRight1 = 5;
int motorRight2 = 6;
int motorLeft1  = 4;
int motorLeft2  = 3;

// ----------------- Sensor- en multiplexer-instellingen -----------------
#define TCAADDR 0x70
int Lpins[4] = {0, 1, 2, 7};
// Sensorindeling: 
//   measurements[0] = frontLeft, [1] = frontRight, [2] = diagLeft (45° links), [3] = diagRight (45° rechts)
int measurements[4] = {-1, -1, -1, -1};

// ----------------- Maze-definities -----------------
// Een 16x16 grid: in elke cel slaan we de muurdata op via bitflags  
// bit 0: muur naar noorden, bit 1: muur naar oost, bit 2: muur naar zuid, bit 3: muur naar west.
uint8_t maze[16][16] = {0};

// Huidige positie en richting van de robot
int x = 0, y = 0;  // Startpositie (0,0)
int dir = 0;       // Richting: 0 = noord, 1 = oost, 2 = zuid, 3 = west  

// Doel (finish) – bijvoorbeeld het centrum
const int goalX = 8;
const int goalY = 8;

// ----------------- Fasen (States) -----------------
enum Phase { 
  SEARCHING,       // langzaam doolhof verkennen en in kaart brengen
  COMPUTE_ROUTE,   // route berekenen
  RACING,          // zo snel mogelijk naar het doel rijden
  FINISHED,        // doel bereikt
  RETURNING,       // langzaam terug naar start
  RETURN_FINISHED  // terugkeer voltooid
};
Phase currentPhase = SEARCHING;

// Timing voor non-blokkerende werking
long previousSensorTime = 0;
int sensorInterval = 50; // sensor-update elke 50 ms

// Variabelen voor celbeweging
long moveStartTime = 0;
long moveDurationSlow = 1200;  // ms per cel in SEARCHING en RETURNING (langzaam)
long moveDurationFast = 400;   // ms per cel in RACING (snel)

// Variabelen voor draaien in kleine stappen (we voeren een 90°-draai in meerdere stappen uit)
int desiredTurn = 0;   // aantal 90°-turns dat nog nodig is
int turnStepCount = 0;
int stepsNeededFor90deg = 5;

// ----------------- Routeplanning -----------------
// We slaan de berekende route op als een sequentie van richtingen (0 = noord, 1 = oost, 2 = zuid, 3 = west)
const int MAX_ROUTE_LENGTH = 128;
uint8_t route[MAX_ROUTE_LENGTH];
int routeLength = 0;
int routeIndex = 0;  // index voor het aflopen van de route

// ----------------- Functies -----------------

// Selecteer een kanaal op de TCA9548A-multiplexer
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Lees de sensorwaarden (non-blokkerend)
void refresh(bool debug) {
  VL53L0X_RangingMeasurementData_t measure;
  for (int i = 0; i < 4; i++) {
    tcaselect(Lpins[i]);
    lox.rangingTest(&measure, false);
    measurements[i] = measure.RangeMilliMeter;
  }
  if(debug) {
    for (int i = 0; i < 4; i++) {
      Serial.println(measurements[i]);
    }
  }
}

// Update de maze voor de huidige cel op basis van sensordata  
// (Bijvoorbeeld: als een van de voorwaartse sensoren < 200 mm meet, staat er een muur in de rijrichting)
void updateMaze() {
  if (measurements[0] < 200 || measurements[1] < 200) {
    maze[x][y] |= (1 << dir);
  }
}

// Kies de volgende richting met de laagste waarde in de buurcel  
int decideDirection() {
  int minVal = 255;
  int nextDir = dir;
  for (int i = 0; i < 4; i++) {
    int nx = x, ny = y;
    if      (i == 0) ny--;      // noord
    else if (i == 1) nx++;      // oost
    else if (i == 2) ny++;      // zuid
    else if (i == 3) nx--;      // west

    if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16) {
      if (maze[nx][ny] < minVal) {
        minVal = maze[nx][ny];
        nextDir = i;
      }
    }
  }
  return nextDir;
}

// Controleer of het doel (goalX, goalY) is bereikt
bool isGoalReached() {
  return (x == goalX && y == goalY);
}

// Bereken de kortste route (via BFS) vanaf een startcel naar een doeldcel  
void computeShortestRoute(int start_x, int start_y, int target_x, int target_y) {
  int dist[16][16];
  int prevDir[16][16]; // slaat op: welke richting ging je in de vorige stap naar deze cel
  for (int i = 0; i < 16; i++)
    for (int j = 0; j < 16; j++) {
      dist[i][j] = 9999;
      prevDir[i][j] = -1;
    }
  dist[start_x][start_y] = 0;
  
  int q[256][2];
  int front = 0;
  int back = 0;
  q[back][0] = start_x;
  q[back][1] = start_y;
  back = (back + 1) % 256;
  
  int dx[4] = {0, 1, 0, -1};
  int dy[4] = {-1, 0, 1, 0};
  
  while (front != back) {
    int cur_x = q[front][0];
    int cur_y = q[front][1];
    front = (front + 1) % 256;
    
    for (int d = 0; d < 4; d++) {
      int nx = cur_x + dx[d];
      int ny = cur_y + dy[d];
      if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16)
        continue;
      // Als er in cel cur in richting d een muur staat, dan is er geen doorgang.
      if (maze[cur_x][cur_y] & (1 << d)) continue;
      
      if (dist[nx][ny] > dist[cur_x][cur_y] + 1) {
        dist[nx][ny] = dist[cur_x][cur_y] + 1;
        prevDir[nx][ny] = d;  // we kwamen in cel (nx,ny) via richting d vanuit cur
        q[back][0] = nx;
        q[back][1] = ny;
        back = (back + 1) % 256;
      }
    }
  }
  
  if (dist[target_x][target_y] == 9999) {
    Serial.println(F("Geen route gevonden!"));
    routeLength = 0;
    return;
  }
  
  // Route reconstrueren (van target naar start)
  int revRoute[MAX_ROUTE_LENGTH];
  int revRouteLength = 0;
  int cx = target_x, cy = target_y;
  while (cx != start_x || cy != start_y) {
    int d = prevDir[cx][cy];
    revRoute[revRouteLength++] = d;
    cx -= dx[d];
    cy -= dy[d];
  }
  // Keer de route om zodat je van start naar target rijdt
  routeLength = revRouteLength;
  for (int i = 0; i < revRouteLength; i++) {
    route[i] = revRoute[revRouteLength - 1 - i];
  }
  
  Serial.print("Route: ");
  for (int i = 0; i < routeLength; i++) {
    Serial.print(route[i]);
    Serial.print(F(" "));
  }
  Serial.println();
}

// ----------------- Motorbesturingsfuncties -----------------

void forward(int speed) {
  analogWrite(motorLeft1, speed);
  analogWrite(motorLeft2, 0);
  analogWrite(motorRight1, 0);
  analogWrite(motorRight2, speed);
}

void reverse(int speed) {
  analogWrite(motorLeft1, 0);
  analogWrite(motorLeft2, speed);
  analogWrite(motorRight1, speed);
  analogWrite(motorRight2, 0);
}

void standStill() {
  analogWrite(motorLeft1, 0);
  analogWrite(motorLeft2, 0);
  analogWrite(motorRight1, 0);
  analogWrite(motorRight2, 0);
}

void turnRightStep() {
  analogWrite(motorLeft1, 150);
  analogWrite(motorLeft2, 0);
  analogWrite(motorRight1, 0);
  analogWrite(motorRight2, 150);
}

// ----------------- Kleine celebratie (optioneel) -----------------
void celebrate() {
  Serial.println("Doel bereikt in zoekfase! Doolhof in kaart.");
  for (int i = 0; i < 3; i++) {
    forward(200);
    delay(200);
    reverse(200);
    delay(200);
  }
  standStill();
}

// ----------------- setup() -----------------
void setup() {
  Serial.begin(9600);
  Serial.println("starting");
  Serial.println("help");
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);

  Wire.begin();
  for (int i = 0; i < 4; i++) {
    tcaselect(Lpins[i]);
    if (!lox.begin()) {
      Serial.println("Failed to boot VL53L0X met sensor " + i);
      while (1);
    }
  }
  Serial.println(F("Setup complete"));
  delay(2000);
  moveStartTime = millis();
}

// ----------------- loop() met state machine -----------------
void loop() {
  // Lees regelmatig de sensoren (zoals eerder)
  if (millis() - previousSensorTime >= sensorInterval) {
    previousSensorTime = millis();
    refresh(false);
    updateMaze();
  }
  
  // Als er een draai nodig is, voeren we die uit (voor alle fasen)
  if (desiredTurn > 0) {
    turnRightStep();
    turnStepCount++;
    if (turnStepCount >= stepsNeededFor90deg) {
      desiredTurn--;
      turnStepCount = 0;
      // Update de globale richting (hier nemen we aan dat we rechts draaien)
      dir = (dir + 1) % 4;
      // Reset de timer voor celbeweging indien nodig
      moveStartTime = millis();
    }
    return;  // Tijdens een draai voeren we geen andere acties uit.
  }
  
  switch (currentPhase) {
    case SEARCHING: {
      int frontLeft = measurements[0];
      int frontRight = measurements[1];
      int diagLeft = measurements[2];
      int diagRight = measurements[3];
      
      int baseSpeed = 120;
      int correction = 0;
      int thresholdDiff = 20;
      if (abs(frontLeft - frontRight) > thresholdDiff) {
        if (frontLeft < frontRight)
          correction = +20;
        else
          correction = -20;
      }
      int diagThreshold = 200;
      if (diagLeft < diagThreshold)  correction += 10;
      if (diagRight < diagThreshold) correction -= 10;
      
      int leftSpeed  = constrain(baseSpeed - correction, 0, 255);
      int rightSpeed = constrain(baseSpeed + correction, 0, 255);
      
      analogWrite(motorLeft1, leftSpeed);
      analogWrite(motorLeft2, 0);
      analogWrite(motorRight1, 0);
      analogWrite(motorRight2, rightSpeed);
      
      if (millis() - moveStartTime >= moveDurationSlow) {
        int nextDir = decideDirection();
        desiredTurn = (nextDir - dir + 4) % 4;
        moveStartTime = millis();
      }
      
      if (isGoalReached()) {
        standStill();
        celebrate();
        currentPhase = COMPUTE_ROUTE;
      }
    }
      break;
      
    case COMPUTE_ROUTE: {
      if (routeLength == 0) {  // route nog niet berekend
        // Voor de RACING-fase: route van start (0,0) naar doel
        computeShortestRoute(0, 0, goalX, goalY);
        // Reset positie en richting voor RACING
        x = 0; y = 0; dir = 1;
        routeIndex = 0;
        moveStartTime = millis();
        currentPhase = RACING;
        Serial.println("Racefase gestart!");
      }
    }
      break;
      
    case RACING: {
      // RACING: de robot rijdt continu met een hogere snelheid, zonder volledig te stoppen.
      int baseSpeed = 200;  // hogere snelheid in RACING
      forward(baseSpeed);
      
      // Zodra de tijd voor één cel is verstreken, werken we de positie bij en passen we de richting aan.
      if (millis() - moveStartTime >= moveDurationFast) {
        if (dir == 0)      y--;
        else if (dir == 1) x++;
        else if (dir == 2) y++;
        else if (dir == 3) x--;
        
        if (routeIndex < routeLength) {
          int desiredDir = route[routeIndex];
          routeIndex++;
          desiredTurn = (desiredDir - dir + 4) % 4;
        }
        moveStartTime = millis();
      }
    }
      break;
      
    case FINISHED: {
      // Na RACING: het doel is bereikt.
      standStill();
      Serial.println(F("Finish bereikt! Bereid terugkeer voor..."));
      // Bereken route terug van finish (goal) naar start (0,0)
      computeShortestRoute(goalX, goalY, 0, 0);
      x = goalX; y = goalY;
      // Stel startrichting voor terug
      dir = 3;
      routeIndex = 0;
      moveStartTime = millis();
      currentPhase = RETURNING;
    }
      break;
      
    case RETURNING: {
      // RETURNING: net als in SEARCHING rijdt de robot langzaam terug.
      int baseSpeed = 120;
      forward(baseSpeed);
      
      if (millis() - moveStartTime >= moveDurationSlow) {
        // Update positie
        if (dir == 0)      y--;
        else if (dir == 1) x++;
        else if (dir == 2) y++;
        else if (dir == 3) x--;
        
        int desiredDir = route[routeIndex];
        routeIndex++;
        desiredTurn = (desiredDir - dir + 4) % 4;
        moveStartTime = millis();
      }
    }
      break;
      
    case RETURN_FINISHED: {
      standStill();
      Serial.println(F("Terugkeer voltooid. Robot staat weer op het startpunt!"));
      while (1);  // Eindstand: robot stopt hier
    }
      break;
  }
}