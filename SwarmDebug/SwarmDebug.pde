/*=========================================
      SWARM ROBOTS ISP DEBUG PROGRAM
            Rayce Stipanovich
=========================================*/

//Include Libraries and Packages
import controlP5.*;
import org.gwoptics.graphics.graph2D.Graph2D;
import org.gwoptics.graphics.graph2D.traces.ILine2DEquation;
import org.gwoptics.graphics.graph2D.traces.RollingLine2DTrace;
import processing.serial.*;
import java.util.Calendar;
import java.text.SimpleDateFormat;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/*=========================================
  Global Defines
=========================================*/
Serial comPort;
ControlP5 cp5;
RollingLine2DTrace r,r2,r3;
Graph2D g;
Textarea consoleTextarea;
DropdownList portChooser;
ScheduledExecutorService worker = Executors.newSingleThreadScheduledExecutor();
public static final String DATE_FORMAT_NOW = "yyyy-MM-dd HH:mm:ss";
int serialPortNumber = -1;
boolean foundSerial = false;
boolean autoConnectSerial = true;
String currentSerialDevice = "";
byte previousByte = 0;
byte lastByte = 0;
byte inByte = 0;
int bytesToRead = 33;
byte[] serialBuffer = new byte[bytesToRead];
int placeInFrame = 0;
long packetNumber = 1;
boolean inPacket = false;
CheckBox checkbox;

/*=========================================
  Defines for Data
=========================================*/
int IR0, IR1, IR3, IR4, IR5, IR6, IR7;
int MEMS0, MEMS1, MEMS2, MEMS3;
int LDR;
boolean ES0, ES1, ES2, ES3;
int IMU_X, IMU_Y, IMU_Z;
int TEMP, CURR, RSSI, VOL0, VOL1;
boolean BS0, BS1, BS2, BS3;
boolean DP0, DP1, DP2, DP3;

/*=========================================
  Functions for Console Logging
=========================================*/
public static String now() {
  Calendar cal = Calendar.getInstance();
  SimpleDateFormat sdf = new SimpleDateFormat(DATE_FORMAT_NOW);
  return sdf.format(cal.getTime());
}

void logConsole(String toadd) {
  String oldtest;
  String date = now();
  oldtest = consoleTextarea.getText();
  if (oldtest != null && oldtest.length() >0) consoleTextarea.setText(oldtest + "  \n\r CONSOLE:  " + date + ":   " + toadd);
  else consoleTextarea.setText("  CONSOLE:  " + date + ":   " + toadd);
  consoleTextarea.scroll(1.0);
}

void logError(String toadd) {
  String oldtest;
  String date = now();
  oldtest = consoleTextarea.getText();
  if (oldtest != null && oldtest.length() >0) consoleTextarea.setText(oldtest + "  \n\r== ERROR:  " + date + ":   " + toadd);
  else consoleTextarea.setText("==  ERROR:  " + date + ":   " + toadd);
  consoleTextarea.scroll(1.0);
}

/*=========================================
  Serial Functions
=========================================*/
void autoConnect() {
  if (autoConnectSerial && !foundSerial) {
    int serialCount = Serial.list().length;
    if (serialCount > 0) {
      portChooser.captionLabel().set(Serial.list()[0]);
      serialPortNumber = 0;
      connectSwarm(0);
    }
  }
}
void resetConnection() {
  portChooser.captionLabel().set("Select Serial Port");
  serialPortNumber = -1;
  if (foundSerial) {
    if (comPort != null) comPort.stop();
    logConsole("Lost Serial Connection to " + currentSerialDevice);
    foundSerial = false;
  }
}

void listSerialPorts() {
  logConsole("Finding SWARM Devices...");
  int serialCount = Serial.list().length;
  if (serialCount == 0) {
    portChooser.clear();
    logError("No Serial Devices Found");
    resetConnection();
    serialPortNumber = -1;
  } else {
   logConsole("Found " + serialCount + " devices!");
   portChooser.clear();
   Serial.list();
    for (int i=0; i < serialCount; i++){
      portChooser.addItem(Serial.list()[i], i);
      logConsole("Found " + Serial.list()[i]);
    }
    autoConnect();
  }
}

public void _listSerialTimer() {
  int serialCount = Serial.list().length;
  if (serialCount == 0) {
   portChooser.clear();
   resetConnection();
   portChooser.captionLabel().set("Select Serial Port");
   serialPortNumber = -1;
  } else {
   portChooser.clear();
   boolean foundOurSerial = false;
   Serial.list();
    for (int i=0; i < serialCount; i++){
      portChooser.addItem(Serial.list()[i], i);
      if ( Serial.list()[i].equals(currentSerialDevice) ) {
        foundOurSerial = true;
      }
    }
    if (!foundOurSerial && foundSerial) resetConnection();
    autoConnect();
  }
  
  //check for device connectivity
  Runnable task = new Runnable() {
    public void run() {
      _listSerialTimer();
    }
  };
  worker.schedule(task, 1000, TimeUnit.MILLISECONDS);
}

public void initializeSerial() {
  listSerialPorts();
  _listSerialTimer();
  _readSerial();
}

void connectSwarm(int serialDevice) {
  if (serialDevice < 0 || Serial.list().length <= serialDevice) {
    logError("Invalid device ID");
    resetConnection();
  } else if (Serial.list().length == 0) {
    logError("No Serial Devices Found");
    resetConnection();
  } else {
    logConsole("Connecting to " + Serial.list()[serialDevice]);
    try {
      if (comPort != null) comPort.stop();
      comPort = new Serial(this, Serial.list()[serialDevice], 9600, 'N', 8, 1.0);
      comPort.buffer(bytesToRead * 3);
      logConsole("Connected!");
      currentSerialDevice = Serial.list()[serialDevice];
      serialPortNumber = serialDevice;
      foundSerial = true;
    } catch (Exception e) {
      //leave blank
      logError("This device is busy!");
      resetConnection();
    }
    if (comPort == null) {
      logError("Error establishing connection.");
      resetConnection();
    }
  }
}

void useAutoConnect(float[] a) {
  autoConnectSerial = (a[0] == 1.0);
}

void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) {
    String listName = theEvent.getName();
    if (listName.equals("portChooserD")) {
      serialPortNumber = (int)portChooser.getValue();
    }
    connectSwarm(serialPortNumber);
  }
}

void parseSerialFrame() {
  
}

void readSerial() {
 if (comPort.available() != 0) {
  previousByte = (byte)lastByte;
  lastByte = (byte)inByte;
  inByte = (byte)comPort.read();
  if (previousByte == 0x3D && lastByte == 0x3D && inByte == 0x3D ) {
    //begin parsing serial...
    if (inPacket && placeInFrame > 3 && placeInFrame < bytesToRead) logError("Dropped Malformed Packet " + (packetNumber-1));
    placeInFrame = 0;
    logConsole("Starting Packet " + packetNumber++);
    inPacket = true;
  }
  if (placeInFrame < bytesToRead) serialBuffer[placeInFrame] = inByte;
  placeInFrame++;
  if (placeInFrame >= bytesToRead) {
    if (inPacket) {
      logConsole("Recieved Packet " + (packetNumber-1));
      parseSerialFrame();
      placeInFrame = 0;
      inPacket = false;
    }
  }
 }
}

void _readSerial() {
  readSerial();
  //check for device connectivity
  Runnable task = new Runnable() {
    public void run() {
      _readSerial();
    }
  };
  worker.schedule(task, 1, TimeUnit.MILLISECONDS);
}

/*=========================================
  Initialize Graphics
=========================================*/
public void setupGraphics() {
  consoleTextarea = cp5.addTextarea("consoleLog").setPosition(10,710).setSize(585,180).setFont(createFont("arial", 12)).setLineHeight(14).setColor(color(200)).setColorBackground(color(20)).setColorForeground(color(20)).showScrollbar().setScrollForeground(color(70)).setScrollBackground(color(0));

  portChooser = cp5.addDropdownList("portChooserD").setPosition(379, 241).setWidth(260);
  portChooser.setBackgroundColor(color(190));
  portChooser.setItemHeight(20);
  portChooser.setBarHeight(20);
  portChooser.captionLabel().set("Select Serial Port");
  portChooser.captionLabel().style().marginTop = 5;
  portChooser.captionLabel().style().marginLeft = 3;
  portChooser.valueLabel().style().marginTop = 3;
  portChooser.setValue(9999.0);
  portChooser.setColorActive(color(255, 128));
  
  cp5.addButton("refreshUSB").setPosition(650, 220).setSize(80, 20).setCaptionLabel("Refresh Devices").captionLabel().style().marginLeft = 2;
  cp5.addButton("connectUSB").setPosition(740, 220).setSize(50, 20).setCaptionLabel("Reset").captionLabel().style().marginLeft = 5;
  cp5.addButton("disconnectUSB").setPosition(730, 250).setSize(60, 20).setCaptionLabel("Disconnect").captionLabel().style().marginLeft = 4;
  
  cp5.addToggle("autoConnectSerial").setPosition(40,100).setSize(50,20).setCaptionLabel("Auto Connect");
  
  cp5.getTooltip().setDelay(500);
  cp5.getTooltip().register("portChooserD","Click on a device to connect.");
  cp5.getTooltip().register("connectUSB","Reset connection to selected Device.");
  cp5.getTooltip().register("disconnectUSB","Disconnect from device.");
}

/*=========================================
  UI Functions
=========================================*/
void disconnectUSB() {
  logConsole("Disconnecting Serial Device..");
  resetConnection();
}
void connectUSB() {
  logConsole("Resetting connection to Serial Device..");
  resetConnection();
  autoConnect();
}

/*=========================================
  Main Initialization
=========================================*/
public void setup() {
  frame.setTitle("SWARM Robotics Debugger");
  PFont font = createFont("arial", 12);
  size(1200,900, P2D);
  noStroke();

  cp5 = new ControlP5(this);
  cp5.setAutoDraw(false);
  setupGraphics();
  logConsole("Graphics Initialized");
  
  logConsole("Initializing Serial Communications");
  initializeSerial();
  
  logConsole("Finished Initialization");
}

/*=========================================
  Global Output Renderer
=========================================*/
public void draw() {
  background(0);
  
  fill(40);
  rect(605, 710, 585, 180);
  
  cp5.draw();
  //g.draw();
}
