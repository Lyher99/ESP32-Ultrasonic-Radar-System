import processing.serial.*;

Serial esp;

int angle = 0;
int dist  = -1;

int maxCM = 200;

// ===== BFS Object (from ESP32) =====
int objSize = 0;        // number of grid cells in cluster
float objCx = -1;       // centroid X in grid units
float objCy = -1;       // centroid Y in grid units

// Must match ESP32 settings (IMPORTANT)
int GRID = 32;
float CELL_CM = 10.0;   // same as ESP32 CELL_CM

void setup() {
  size(900, 520);
  smooth();

  println("=== Serial Ports ===");
  println(Serial.list());

  // Change index if needed
  esp = new Serial(this, Serial.list()[0], 115200);
  esp.bufferUntil('\n');
}

void draw() {
  background(0);

  // Radar base
  int cx = width/2;
  int cy = height - 30;
  int R  = height - 70;

  stroke(0, 255, 0, 60);
  strokeWeight(2);

  // arcs
  for (int i=1; i<=4; i++) {
    noFill();
    arc(cx, cy, (R*i/2), (R*i/2), PI, TWO_PI);
  }

  // angle lines
  for (int a=0; a<=180; a+=30) {
    float rad = radians(a);
    line(cx, cy, cx - cos(rad)*R, cy - sin(rad)*R);
    fill(0,255,0,120);
    textAlign(CENTER, CENTER);
    text(a + "°", cx - cos(rad)*(R+25), cy - sin(rad)*(R+25));
    noFill();
  }

  // sweep line
  stroke(0, 255, 0);
  strokeWeight(3);
  float angRad = radians(angle);
  line(cx, cy, cx - cos(angRad)*R, cy - sin(angRad)*R);

  // dot from raw distance
  if (dist > 0 && dist <= maxCM) {
    float rr = map(dist, 0, maxCM, 0, R);
    float x = cx - cos(angRad)*rr;
    float y = cy - sin(angRad)*rr;

    noStroke();
    fill(255, 60, 60);
    ellipse(x, y, 10, 10);
  }

  // ===== Draw BFS Object centroid (yellow) =====
  // ESP sends centroid in grid units (0..GRID-1)
  if (objSize > 0 && objCx >= 0 && objCy >= 0) {

    // Convert (grid cx,cy) -> cm -> pixels
    float xcm = objCx * CELL_CM;
    float ycm = objCy * CELL_CM;

    // Convert to polar-ish view like our radar:
    // In ESP code: x = dist*cos(angle), y = dist*sin(angle)
    // Here we interpret:
    //   distance = sqrt(x^2+y^2)
    //   angle = atan2(y, x)
    float d = sqrt(xcm*xcm + ycm*ycm);
    float a = atan2(ycm, xcm); // radians

    float rr = map(d, 0, maxCM, 0, R);
    float x = cx - cos(a)*rr;
    float y = cy - sin(a)*rr;

    noStroke();
    fill(255, 220, 0);
    ellipse(x, y, 14, 14);

    // little label
    fill(255, 220, 0);
    textAlign(CENTER, BOTTOM);
    textSize(12);
    text("OBJ (" + objSize + ")", x, y - 10);
  }

  // text info
  fill(0, 255, 0);
  textAlign(LEFT, TOP);
  textSize(18);
  text("Angle: " + angle + "°", 20, 20);
  text("Distance: " + ((dist < 0) ? "--" : dist + " cm"), 20, 45);

  textSize(14);
  text("Best Object: " + (objSize > 0 ? ("size=" + objSize) : "--"), 20, 75);
}

void serialEvent(Serial p) {
  String line = trim(p.readStringUntil('\n'));
  if (line == null || line.length() == 0) return;

  // Two formats:
  // 1) angle,distance
  // 2) OBJ,size,cx,cy

  if (line.startsWith("OBJ,")) {
    String[] parts = split(line, ',');
    if (parts.length == 4) {
      try {
        objSize = int(parts[1]);
        objCx   = float(parts[2]);
        objCy   = float(parts[3]);
      } catch(Exception e) {
        // ignore bad OBJ line
      }
    }
    return;
  }

  // Normal: angle,distance
  String[] parts = split(line, ',');
  if (parts.length == 2) {
    try {
      angle = int(parts[0]);
      dist  = int(parts[1]);
    } catch(Exception e) {
      // ignore bad line
    }
  }
}
