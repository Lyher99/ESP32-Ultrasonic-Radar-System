#include <ESP32Servo.h>

/* ================= PIN SETTINGS ================= */
#define TRIG_PIN  5
#define ECHO_PIN  18
#define SERVO_PIN 13

/* ================= RADAR SETTINGS ================= */
const int ANGLE_MIN = 0;
const int ANGLE_MAX = 180;
const int MAX_DISTANCE_CM = 200;

const int SERVO_STEP_DEG = 2;
const uint32_t MOVE_MS   = 60;
const uint32_t SETTLE_MS = 25;
const uint32_t READ_MS   = 90;

Servo radarServo;

int currentAngle = ANGLE_MIN;
int currentDistance = -1;
int servoDirection = 1;

uint32_t lastMoveMs = 0;
uint32_t lastReadMs = 0;
uint32_t movedAtMs  = 0;

/* ================= GRID MAP (for BFS) ================= */
const int GRID = 32;                 // 32x32
uint8_t occ[GRID][GRID];             // 0 empty, 1 occupied
const float CELL_CM = 10.0;          // 1 cell = 10cm  (32*10 = 320cm range coverage)

// Queue for BFS
struct Pt { uint8_t x, y; };
Pt q[GRID * GRID];

/* ================= ULTRASONIC ================= */
int readUltrasonicOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);
  if (duration == 0) return -1;

  int cm = (int)(duration * 0.0343f / 2.0f);
  if (cm < 2 || cm > MAX_DISTANCE_CM) return -1;
  return cm;
}

int readUltrasonicMedian3() {
  int a = readUltrasonicOnce(); delay(2);
  int b = readUltrasonicOnce(); delay(2);
  int c = readUltrasonicOnce();

  if (a < 0 && b < 0 && c < 0) return -1;

  int A = (a < 0) ? 9999 : a;
  int B = (b < 0) ? 9999 : b;
  int C = (c < 0) ? 9999 : c;

  int m;
  if ((A <= B && B <= C) || (C <= B && B <= A)) m = B;
  else if ((B <= A && A <= C) || (C <= A && A <= B)) m = A;
  else m = C;

  return (m == 9999) ? -1 : m;
}

/* ================= HELPERS ================= */
void clearGrid() {
  for (int y = 0; y < GRID; y++) {
    for (int x = 0; x < GRID; x++) occ[y][x] = 0;
  }
}

// Put one occupied point into grid from polar (angle, distance)
void addToGrid(int angleDeg, int distCm) {
  if (distCm < 0) return;

  // Convert to radians and cartesian (cm)
  float rad = angleDeg * 3.1415926f / 180.0f;
  float xcm = distCm * cosf(rad);
  float ycm = distCm * sinf(rad);

  // Shift origin to grid center-left (like radar at left middle)
  // You can change this mapping depending on your UI.
  int gx = (int)(xcm / CELL_CM);
  int gy = (int)(ycm / CELL_CM);

  // Clamp into grid
  if (gx < 0) gx = 0;
  if (gy < 0) gy = 0;
  if (gx >= GRID) gx = GRID - 1;
  if (gy >= GRID) gy = GRID - 1;

  occ[gy][gx] = 1;
}

// BFS flood-fill to get one cluster size + centroid sum
int bfsCluster(uint8_t sx, uint8_t sy, uint8_t visited[GRID][GRID], int &sumX, int &sumY) {
  int head = 0, tail = 0;
  q[tail++] = {sx, sy};
  visited[sy][sx] = 1;

  int count = 0;
  sumX = 0;
  sumY = 0;

  while (head < tail) {
    Pt p = q[head++];
    count++;
    sumX += p.x;
    sumY += p.y;

    // 4-neighbors
    const int dx[4] = {1,-1,0,0};
    const int dy[4] = {0,0,1,-1};

    for (int i = 0; i < 4; i++) {
      int nx = (int)p.x + dx[i];
      int ny = (int)p.y + dy[i];
      if (nx < 0 || ny < 0 || nx >= GRID || ny >= GRID) continue;
      if (visited[ny][nx]) continue;
      if (occ[ny][nx] == 0) continue;

      visited[ny][nx] = 1;
      q[tail++] = {(uint8_t)nx, (uint8_t)ny};
    }
  }
  return count;
}

// Run BFS over grid and print objects
void detectObjectsBFS() {
  uint8_t visited[GRID][GRID] = {0};

  int bestSize = 0;
  float bestCx = 0, bestCy = 0;

  for (int y = 0; y < GRID; y++) {
    for (int x = 0; x < GRID; x++) {
      if (occ[y][x] == 1 && !visited[y][x]) {
        int sumX, sumY;
        int size = bfsCluster(x, y, visited, sumX, sumY);

        // ignore tiny noise clusters
        if (size < 2) continue;

        float cx = (float)sumX / (float)size;
        float cy = (float)sumY / (float)size;

        if (size > bestSize) {
          bestSize = size;
          bestCx = cx;
          bestCy = cy;
        }
      }
    }
  }

  // Send "best object" to Serial (for Processing)
  // Example: OBJ,size,cx,cy
  Serial.print("OBJ,");
  Serial.print(bestSize);
  Serial.print(",");
  Serial.print(bestCx, 2);
  Serial.print(",");
  Serial.println(bestCy, 2);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);
  radarServo.write(currentAngle);

  clearGrid();
  Serial.println("READY");
}

void loop() {
  uint32_t now = millis();

  // Move servo
  if (now - lastMoveMs >= MOVE_MS) {
    lastMoveMs = now;

    currentAngle += servoDirection * SERVO_STEP_DEG;

    // Detect end of sweep -> run BFS + clear grid
    if (currentAngle >= ANGLE_MAX) {
      currentAngle = ANGLE_MAX;
      servoDirection = -1;

      detectObjectsBFS();
      clearGrid();
    } else if (currentAngle <= ANGLE_MIN) {
      currentAngle = ANGLE_MIN;
      servoDirection = 1;

      detectObjectsBFS();
      clearGrid();
    }

    radarServo.write(currentAngle);
    movedAtMs = now;
  }

  // Read sensor
  if ((now - movedAtMs >= SETTLE_MS) && (now - lastReadMs >= READ_MS)) {
    lastReadMs = now;
    currentDistance = readUltrasonicMedian3();

    // Add to grid for BFS map
    addToGrid(currentAngle, currentDistance);

    // Keep your normal stream for radar draw
    Serial.print(currentAngle);
    Serial.print(",");
    Serial.println(currentDistance);
  }
}
